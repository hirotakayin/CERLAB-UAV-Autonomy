#!/usr/bin/env python3
"""Fly exploration presets and EPIC on the pits several times over, without any GUI, and keep every result.

Each slot has its own ROS master and Gazebo master, so slots share nothing. A fresh sim is started for
every run. Each run gets a folder in the output folder, and results.csv has one row per run, including
runs that crashed or never landed: those write no mission CSV of their own. summarize_runs.py turns the
mission CSVs into the numbers for each run.

  rosrun autonomous_flight run_batch.py --dry-run
  rosrun autonomous_flight run_batch.py                          # every pit with baseline, gain and EPIC, 3 times each
  rosrun autonomous_flight run_batch.py --maps excavation1 excavation_slab --presets gain epic --repeats 3
  rosrun autonomous_flight run_batch.py --presets gain --repeats 1 --parallel 1 --speed 2
  rosrun autonomous_flight run_batch.py --sensor omni --presets baseline gain --out ~/mission_runs/batch_x   # 360 camera, same batch

Every run is recorded from the dashboard, without voxels (record_dashboard.py's paths view), to
<run>/<run>_paths.mp4; --no-record skips it. Log lines are stamped with the wall clock and clock.csv pairs it with the
mission clock, so run_highlights.py can list the notable moments of every run with their place in the video.

epic needs both workspaces sourced: source ~/EPIC/devel/setup.bash; source ~/catkin_ws/devel/setup.bash --extend

Run it again with the same --out to carry on: runs already in results.csv are skipped.
"""
import argparse
import csv
import glob
import json
import os
import queue
import shutil
import signal
import socket
import subprocess
import threading
import time

import yaml

CFG = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'cfg', 'dynamic_exploration')
PRESETS = sorted(os.path.splitext(f)[0] for f in os.listdir(os.path.join(CFG, 'presets')) if f.endswith('.yaml')) + ['epic']
MAPS = sorted(os.path.splitext(f)[0] for f in os.listdir(os.path.join(CFG, 'maps')) if f.endswith('.yaml') and not f.endswith('_mapping.yaml'))
ROS_PORT = 11411 # not the defaults, so a sim started by hand is left alone
GAZEBO_PORT = 11545
FIELDS = ['run', 'map', 'preset', 'repeat', 'slot', 'status', 'wall_s', 'mission_s', 'mean_rtf', 'min_rtf',
          'floor_pct', 'block_pct', 'surface_pct', 'last_state', 'csv', 'video', 'sensor']
POLL_SECONDS = 10
BRIDGE_PORT = 9190 # the recording's rosbridge; not 9090, so a dashboard started by hand is left alone
PAGE_PORT = 8790
RECORDER = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'record_dashboard.py')
RETURN_GRACE_SECONDS = 900 # wall clock allowed to get home after a timeout

stop_event = threading.Event()
results_lock = threading.Lock()
epic_lock = threading.Lock() # EPIC writes its route solver's files to one shared folder: one EPIC run at a time


def schedule(maps, presets, repeats):
    # every round flies each map and preset once, in a rotated order, so none always gets the same slot or neighbour
    pairs = [(m, p) for m in maps for p in presets]
    runs = []
    for r in range(repeats):
        shift = r % len(pairs)
        for m, preset in pairs[shift:] + pairs[:shift]:
            runs.append((m, preset, r + 1))
    return runs


SENSOR = 'lidar' # --sensor; runs with another sensor are named <map>_<preset>_<sensor>_r<n>, so they can share a batch


def run_name(run):
    pit, preset, repeat = run
    return '%s_%s_r%d' % run if SENSOR == 'lidar' else '%s_%s_%s_r%d' % (pit, preset, SENSOR, repeat)


def set_speed(env, speed):
    """Raises Gazebo's real-time limit. Returns '' when done, else why not."""
    code = ('import rospy\n'
            'from gazebo_msgs.srv import GetPhysicsProperties, SetPhysicsProperties\n'
            "rospy.wait_for_service('/gazebo/set_physics_properties', 30)\n"
            "g = rospy.ServiceProxy('/gazebo/get_physics_properties', GetPhysicsProperties)()\n"
            "r = rospy.ServiceProxy('/gazebo/set_physics_properties', SetPhysicsProperties)(g.time_step, %f / g.time_step, g.gravity, g.ode_config)\n"
            "print('ok' if r.success else r.status_message)\n" % speed)
    out = command(env, ['python3', '-c', code], 60).strip()
    return '' if out.endswith('ok') else (out or 'no answer from Gazebo')


def port_in_use(port):
    with socket.socket() as s:
        return s.connect_ex(('127.0.0.1', port)) == 0


def command(env, args, timeout):
    """Output of a short ROS command, or '' if it failed or timed out."""
    try:
        return subprocess.run(args, env=env, timeout=timeout, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                              universal_newlines=True, start_new_session=True).stdout
    except subprocess.TimeoutExpired:
        return ''


def parse_status(text):
    try:
        return json.loads(next(yaml.safe_load_all(text))['data'])
    except (ValueError, TypeError, KeyError, StopIteration, yaml.YAMLError):
        return None


def read_status(env):
    return parse_status(command(env, ['rostopic', 'echo', '-n', '1', '/exploration/status'], 15))


def wait_until(check, timeout, alive):
    end = time.time() + timeout
    while time.time() < end and not stop_event.is_set() and alive():
        if check():
            return True
        time.sleep(2)
    return False


def mission_csvs(run_dir):
    return sorted(glob.glob(os.path.join(run_dir, '*_run[0-9]*.csv'))) # not clock.csv


def start_logged(args, env, path):
    """Starts a process with its output in path, each line stamped with the wall clock: the planners print without times."""
    log = open(path, 'w', buffering=1)
    process = subprocess.Popen(args, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, start_new_session=True)
    def copy():
        for raw in iter(process.stdout.readline, b''):
            log.write('%.3f %s' % (time.time(), raw.decode('utf-8', 'replace')))
        log.close()
    threading.Thread(target=copy, daemon=True).start()
    return process


def start_recording(args, env, slot, run_dir):
    """A rosbridge for this run and a hidden browser recording the dashboard without voxels. Returns (bridge, recorder)."""
    port = BRIDGE_PORT + slot
    bridge = start_logged(['roslaunch', '-p', str(ROS_PORT + slot), 'rosbridge_server', 'rosbridge_websocket.launch', 'port:=%d' % port],
                          env, os.path.join(run_dir, 'rosbridge.log'))
    if not wait_until(lambda: port_in_use(port), 60, lambda: bridge.poll() is None):
        print('[slot %d] no rosbridge on port %d: this run is not recorded' % (slot, port), flush=True)
        return bridge, None
    recorder = start_logged(['python3', RECORDER, '--views', 'paths', '--size', args.record_size, '--port', str(PAGE_PORT + slot),
                             '--ws', 'ws://localhost:%d' % port, '--out', run_dir, '--name', os.path.basename(run_dir)],
                            env, os.path.join(run_dir, 'record.log'))
    return bridge, recorder


def stop_recording(recorder, landed):
    """Lets the page finish (it stops 5 s after landing), then the recorder converts the video to MP4."""
    if recorder is None or recorder.poll() is not None: return
    if landed: time.sleep(15)
    recorder.send_signal(signal.SIGINT)


def stop_launch(process):
    # roslaunch shuts its own nodes down on SIGINT and escalates by itself
    if process is None or process.poll() is not None:
        return
    process.send_signal(signal.SIGINT)
    try:
        process.wait(40)
    except subprocess.TimeoutExpired:
        process.terminate()


def kill_leftovers(marker):
    # throttle_topics.sh leaves background processes behind, and nodes can outlive roslaunch. Only processes
    # carrying this run's own environment marker are touched
    needle = ('CERLAB_BATCH_RUN=' + marker).encode()
    def matching():
        found = []
        for path in glob.glob('/proc/[0-9]*/environ'):
            try:
                with open(path, 'rb') as f:
                    if needle in f.read().split(b'\0'):
                        found.append(int(path.split('/')[2]))
            except OSError:
                pass
        return found
    for sig, wait in ((signal.SIGTERM, 5), (signal.SIGKILL, 1)):
        for pid in matching():
            try:
                os.kill(pid, sig)
            except OSError:
                pass
        time.sleep(wait)


def fly(args, pit, preset, repeat, slot, run_dir):
    """One mission from a fresh sim. Returns the row for results.csv."""
    os.makedirs(run_dir, exist_ok=True)
    marker = '%s_%d' % (os.path.basename(run_dir), os.getpid())
    env = dict(os.environ,
               ROS_MASTER_URI='http://localhost:%d' % (ROS_PORT + slot),
               GAZEBO_MASTER_URI='http://localhost:%d' % (GAZEBO_PORT + slot),
               ROS_IP='127.0.0.1',
               ROS_LOG_DIR=os.path.join(run_dir, 'roslogs'),
               CERLAB_BATCH_RUN=marker)
    row = dict.fromkeys(FIELDS, '')
    row.update(run=os.path.basename(run_dir), map=pit, preset=preset, repeat=repeat, slot=slot, sensor=args.sensor)
    sim = planner = bridge = recorder = None
    sim_log, planner_log = os.path.join(run_dir, 'sim.log'), os.path.join(run_dir, 'planner.log')
    start_time = time.time()
    rtf = []
    try:
        if preset == 'epic':
            # EPIC's simulator, planner and the scoring node come up together in one launch
            sim = start_logged(['roslaunch', '-p', str(ROS_PORT + slot), 'epic_planner', 'pit_1ms.launch', 'map:=' + pit,
                                'gui:=false', 'run_save_dir:=' + run_dir], env, sim_log)
            odom = ['rostopic', 'echo', '-n', '1', '/quad_0/lidar_slam/odom']
        else:
            sim_args = ['roslaunch', '-p', str(ROS_PORT + slot), 'uav_simulator', 'start_%s.launch' % pit, 'gui:=false', 'keyboard:=false',
                        'sensor:=' + args.sensor]
            if pit == 'excavation1': sim_args.append('crane:=%s' % str(args.crane).lower())
            sim = start_logged(sim_args, env, sim_log)
            odom = ['rostopic', 'echo', '-n', '1', '/CERLAB/quadcopter/odom']
        if not wait_until(lambda: command(env, odom, 15), 180, lambda: sim.poll() is None):
            row['status'] = 'aborted' if stop_event.is_set() else 'sim_failed'
            return row

        if preset == 'epic':
            planner = sim
        else:
            if args.speed != 1.0:
                problem = set_speed(env, args.speed)
                if problem:
                    row['status'] = 'speed_failed: ' + problem
                    return row
            command(env, ['rosparam', 'set', '/autonomous_flight/run_save_dir', run_dir], 15)
            planner = start_logged(['roslaunch', 'autonomous_flight', 'dynamic_exploration.launch', 'map:=' + pit, 'preset:=' + preset, 'gui:=false',
                                    'sensor:=' + args.sensor],
                                   env, planner_log)
        if args.record:
            bridge, recorder = start_recording(args, env, slot, run_dir)
        alive = lambda: sim.poll() is None and planner.poll() is None
        start = lambda: 'success: True' in command(env, ['rosservice', 'call', '/exploration/start'], 15)
        if not wait_until(start, 180, alive):
            row['status'] = 'aborted' if stop_event.is_set() else 'start_failed'
            return row

        start_time = time.time()
        with open(os.path.join(run_dir, 'run_info.json'), 'w') as f:
            json.dump({'map': pit, 'preset': preset, 'repeat': repeat, 'mission_start_wall': start_time, 'speed': args.speed}, f)
        clock = open(os.path.join(run_dir, 'clock.csv'), 'w', buffering=1) # wall clock against the mission clock
        clock.write('wall,elapsed,state,block_pct\n')
        status = None
        timed_out_at = None
        last_elapsed, last_wall = None, None
        while True:
            if stop_event.is_set():
                row['status'] = 'aborted'
                break
            if mission_csvs(run_dir):
                time.sleep(3) # the node writes the CSV and then the picture
                row['status'] = 'landed' if timed_out_at is None else 'timeout_then_landed'
                break
            if not alive():
                row['status'] = 'crashed'
                break
            new_status = read_status(env)
            wall = time.time()
            if new_status:
                status = new_status
                clock.write('%.2f,%.2f,%s,%.1f\n' % (wall, status['elapsed'], status['state'], status['explored_block_pct']))
                if last_elapsed is not None and status['elapsed'] > last_elapsed and wall > last_wall:
                    rtf.append((status['elapsed'] - last_elapsed) / (wall - last_wall))
                last_elapsed, last_wall = status['elapsed'], wall
            over_limit = (status and status['elapsed'] >= args.timeout) or wall - start_time >= 2 * args.timeout
            if timed_out_at is None and over_limit:
                timed_out_at = wall
                command(env, ['rosservice', 'call', '/exploration/return_home'], 15)
            if timed_out_at is not None and wall - timed_out_at >= RETURN_GRACE_SECONDS:
                row['status'] = 'timeout_no_landing'
                break
            time.sleep(POLL_SECONDS)

        status = read_status(env) or status
        if status:
            row.update(mission_s=round(status['elapsed'], 1), floor_pct=round(status['floor_pct'], 1),
                       block_pct=round(status['explored_block_pct'], 1), surface_pct=round(status['surface_pct'], 1),
                       last_state=status['state'])
        csvs = mission_csvs(run_dir)
        row['csv'] = os.path.basename(csvs[0]) if csvs else ''
        clock.close()
        return row
    finally:
        row['wall_s'] = round(time.time() - start_time)
        if rtf:
            row['mean_rtf'] = round(sum(rtf) / len(rtf), 2)
            row['min_rtf'] = round(min(rtf), 2)
        stop_recording(recorder, row['status'] in ('landed', 'timeout_then_landed'))
        if planner is not sim: stop_launch(planner)
        stop_launch(sim)
        if recorder is not None:
            try:
                recorder.wait(900) # converting to MP4
            except subprocess.TimeoutExpired:
                pass
        stop_launch(bridge)
        kill_leftovers(marker)
        videos = sorted(glob.glob(os.path.join(run_dir, '*_paths*.mp4')))
        row['video'] = os.path.basename(videos[0]) if videos else ''
        if row['status'] == 'landed':
            shutil.rmtree(env['ROS_LOG_DIR'], ignore_errors=True)


def slot_worker(args, slot, runs, results_path):
    time.sleep(slot * 20) # Gazebo start-ups are the heaviest part: do not start them together
    while not stop_event.is_set():
        try:
            pit, preset, repeat = runs.get_nowait()
        except queue.Empty:
            return
        name = run_name((pit, preset, repeat))
        print('[slot %d] start %s' % (slot, name), flush=True)
        if preset == 'epic':
            with epic_lock:
                row = fly(args, pit, preset, repeat, slot, os.path.join(args.out, name))
        else:
            row = fly(args, pit, preset, repeat, slot, os.path.join(args.out, name))
        print('[slot %d] %s: %s, floor %s%%, %s s, real-time factor mean %s min %s' % (
            slot, name, row['status'], row['floor_pct'], row['mission_s'], row['mean_rtf'], row['min_rtf']), flush=True)
        if row['status'] == 'aborted': # not a result: flown again when the batch is resumed
            shutil.rmtree(os.path.join(args.out, name), ignore_errors=True)
            continue
        with results_lock:
            is_new = not os.path.exists(results_path)
            with open(results_path, 'a', newline='') as f:
                writer = csv.DictWriter(f, FIELDS)
                if is_new:
                    writer.writeheader()
                writer.writerow(row)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--maps', nargs='+', default=MAPS, choices=MAPS)
    parser.add_argument('--presets', nargs='+', default=['baseline', 'gain', 'epic'], choices=PRESETS, help='epic flies EPIC instead of a DEP preset')
    parser.add_argument('--repeats', type=int, default=3)
    parser.add_argument('--parallel', type=int, default=1, help='runs at once. More than 1 slows each sim and changes the planners\' timings')
    parser.add_argument('--timeout', type=float, default=1200, help='mission seconds (sim time) before it is sent home')
    parser.add_argument('--no-record', dest='record', action='store_false', help='skip the dashboard videos')
    parser.add_argument('--record-size', default='1280x720', help='video size, WIDTHxHEIGHT')
    parser.add_argument('--crane', action='store_true', help='add the crane load to excavation1')
    parser.add_argument('--speed', type=float, default=1.0, help='Gazebo real-time limit, e.g. 2 or 4. Only reached if the computer keeps up: see mean_rtf. EPIC always runs in real time')
    parser.add_argument('--out', default=os.path.expanduser('~/mission_runs/batch_' + time.strftime('%Y%m%d_%H%M%S')))
    parser.add_argument('--sensor', default='lidar', choices=['lidar', 'omni'],
                        help='omni: the ideal 360 optic-flow camera (opti360) instead of the Mid-360, for DEP presets; runs are named ..._omni_r<n>')
    parser.add_argument('--dry-run', action='store_true', help='print the runs and exit')
    args = parser.parse_args()
    args.out = os.path.abspath(args.out)
    global SENSOR
    SENSOR = args.sensor
    if args.sensor != 'lidar' and 'epic' in args.presets:
        print('EPIC flies its own simulator and lidar: --sensor %s works with the DEP presets only' % args.sensor)
        return 1

    results_path = os.path.join(args.out, 'results.csv')
    done = set()
    if os.path.exists(results_path):
        with open(results_path, newline='') as f:
            done = {row['run'] for row in csv.DictReader(f)}
    todo = [r for r in schedule(args.maps, args.presets, args.repeats) if run_name(r) not in done]

    print('%d runs, %d at a time, output %s' % (len(todo), args.parallel, args.out))
    for i, run in enumerate(todo):
        print('  %2d. %s' % (i + 1, run_name(run)))
    if args.dry_run:
        return 0

    if 'epic' in args.presets and command(dict(os.environ), ['rospack', 'find', 'epic_planner'], 15).strip() == '':
        print('epic needs EPIC on the package path: source ~/EPIC/devel/setup.bash; source ~/catkin_ws/devel/setup.bash --extend')
        return 1
    if not os.environ.get('DISPLAY'):
        print('No DISPLAY: the simulated depth camera and EPIC\'s lidar need one, even with no windows open. Run this from a desktop terminal.')
        return 1
    busy = [p for s in range(args.parallel) for p in (ROS_PORT + s, GAZEBO_PORT + s, BRIDGE_PORT + s, PAGE_PORT + s) if port_in_use(p)]
    if busy:
        print('Ports in use: %s. Is another batch running?' % busy)
        return 1

    os.makedirs(args.out, exist_ok=True)
    runs = queue.Queue()
    for run in todo:
        runs.put(run)
    workers = [threading.Thread(target=slot_worker, args=(args, s, runs, results_path)) for s in range(args.parallel)]
    for worker in workers:
        worker.start()
    try:
        while any(w.is_alive() for w in workers):
            time.sleep(1)
    except KeyboardInterrupt:
        print('Stopping: shutting the running sims down, this takes up to a minute')
        stop_event.set()
    for worker in workers:
        worker.join()
    print('Results: ' + results_path)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
