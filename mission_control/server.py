#!/usr/bin/env python3
"""CERLAB UAV Mission Control backend: starts and stops the launch files behind each button, for the simulator (this laptop) and
the real drone (over SSH), and keeps logs and bags inside a disk budget. Serves the page in web/ and a small JSON API.
Python 3.8 standard library only.

  ./mission_control.sh              # starts this and opens the window
  python3 server.py [--port 8765] [--config config.json]

Local tasks run in their own process group; Stop sends Ctrl-C (SIGINT) to the group, then SIGTERM after 15 s, then
SIGKILL. Drone tasks run in tmux sessions on the drone, so they keep running if WiFi drops; Stop sends Ctrl-C to the
session. SSH uses one shared connection (ControlMaster) and needs key login (ssh-copy-id) to the drone.

The Simulator task starts uav_simulator/start_world.launch with the world and drone picked in the galleries
(/api/worlds, /api/drones). Force quit kills a task's processes at once. /api/term/* gives the page real bash
terminals with the tab's ROS setup, for debugging what is running.

Disk: each task's log is capped (log_file_cap_mb, one rotation), all logs share log_budget_mb (oldest sessions are
deleted first), ROS's own node logs go into the session folder too, bags are split into bag_split_mb pieces and
lz4-compressed, a recording will not start below min_free_gb + one piece, and a running one is stopped cleanly when
free space falls below min_free_gb.
"""
import argparse
import base64
import collections
import json
import os
import re
import shlex
import shutil
import socket
import signal
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

import assets
import runs as history
from terminals import Terminals

HERE = os.path.dirname(os.path.abspath(__file__))
ANSI = re.compile(r'\x1b\][^\x07]*\x07|\x1b\[[0-9;?]*[A-Za-z]')   # colours and terminal-title codes
GB = 1024 ** 3
MB = 1024 ** 2


def expand(p):
    return os.path.abspath(os.path.expanduser(p))


def dir_size(path):
    total = 0
    for root, _, files in os.walk(path):
        for f in files:
            try: total += os.path.getsize(os.path.join(root, f))
            except OSError: pass
    return total


class CappedLog:
    """Last lines in memory for the page, all lines to a file capped at cap bytes (one rotation to .1)."""
    def __init__(self, path, cap):
        self.path, self.cap, self.lines, self.lock = path, cap, collections.deque(maxlen=2000), threading.Lock()
        os.makedirs(os.path.dirname(path), exist_ok=True)
        self.f = open(path, 'a', buffering=1, errors='replace')

    def write(self, line):
        line = ANSI.sub('', line)
        with self.lock:
            self.lines.append(line.rstrip('\n'))
            self.f.write(line if line.endswith('\n') else line + '\n')
            if self.f.tell() > self.cap:
                self.f.close()
                os.replace(self.path, self.path + '.1')
                self.f = open(self.path, 'a', buffering=1, errors='replace')

    def tail(self, n):
        with self.lock: return list(self.lines)[-n:]


class Task:
    def __init__(self, group, spec):
        self.group, self.spec, self.id = group, spec, spec['id']
        self.key = group.name + '/' + self.id
        self.state, self.code, self.started, self.variant = 'stopped', None, None, 0
        self.message = ''
        self.choice = {}                                    # world / drone the simulator was started with
        self.lock = threading.Lock()

    @property
    def remote(self):
        return self.group.name == 'drone' and not self.spec.get('local')

    def variants(self):
        return self.spec.get('variants') or [{'label': '', 'cmd': self.spec.get('cmd', '')}]

    def configured(self):
        return bool(self.spec.get('record')) or any(v.get('cmd') for v in self.variants())

    def info(self):
        return {'id': self.id, 'label': self.spec['label'], 'detail': self.spec.get('detail', ''), 'state': self.state,
                'code': self.code, 'since': self.started, 'variant': self.variant, 'message': self.message,
                'variants': [v['label'] for v in self.variants()] if self.spec.get('variants') else [],
                'variant_info': [{k: v.get(k) for k in ('mode', 'about', 'details', 'learned', 'own_sim', 'dir', 'manual')} for v in self.variants()]
                                if self.spec.get('variants') else [],
                'needs': self.spec.get('needs', []), 'advanced': bool(self.spec.get('advanced')),
                'record': bool(self.spec.get('record')), 'configured': self.configured(), 'remote': self.remote,
                'hidden': bool(self.spec.get('hidden')), 'sizes': [v.get('size', '') for v in self.variants()],
                'choice': self.choice}


class App:
    def __init__(self, cfg_path):
        self.cfg = json.load(open(cfg_path))
        st = self.cfg['storage']
        self.root = expand(st['root'])
        self.bag_dir = os.path.join(self.root, 'bags')
        self.session = time.strftime('%Y%m%d_%H%M%S')
        self.log_dir = os.path.join(self.root, 'logs', self.session)
        os.makedirs(self.bag_dir, exist_ok=True); os.makedirs(self.log_dir, exist_ok=True)
        self.cap, self.budget = st['log_file_cap_mb'] * MB, st['log_budget_mb'] * MB
        self.min_free, self.split_mb = st['min_free_gb'] * GB, st['bag_split_mb']
        self.groups = {}
        for name in ('sim', 'drone'):
            g = type('Group', (), {})(); g.name, g.cfg = name, self.cfg[name]
            g.tasks = collections.OrderedDict((t['id'], Task(g, t)) for t in g.cfg['tasks'])
            self.groups[name] = g
        self.procs, self.logs = {}, {}
        self.notices = collections.deque(maxlen=20)
        self.drone = {'connected': False, 'checked': None, 'free_gb': None, 'message': 'Not connected', 'sessions': []}
        self.bag_rate = 0.0; self._last_bag = (time.time(), self.bag_bytes())
        self.ctl_path = os.path.join('/tmp', 'mc-ssh-%d' % os.getuid())
        sim_pkg = expand(self.cfg['sim'].get('simulator_dir', os.path.join(HERE, '..', 'uav_simulator')))
        self.worlds_dir, self.urdf_dir = os.path.join(sim_pkg, 'worlds'), os.path.join(sim_pkg, 'urdf')
        self.terms = Terminals()
        self.prune_logs()
        threading.Thread(target=self.watch, daemon=True).start()

    # ------------------------------------------------------------------ helpers
    def notice(self, level, text):
        self.notices.append({'t': time.time(), 'level': level, 'text': text})

    def log(self, task):
        if task.key not in self.logs:
            self.logs[task.key] = CappedLog(os.path.join(self.log_dir, task.group.name + '_' + task.id + '.log'), self.cap)
        return self.logs[task.key]

    def free_bytes(self):
        return shutil.disk_usage(self.root).free

    def bag_bytes(self):
        return dir_size(self.bag_dir)

    def prune_logs(self):
        """Delete the oldest log sessions (never the current one) until all logs fit the budget."""
        base = os.path.join(self.root, 'logs')
        sessions = sorted(d for d in os.listdir(base) if d != self.session)
        sizes = {d: dir_size(os.path.join(base, d)) for d in sessions}
        total = sum(sizes.values()) + dir_size(self.log_dir)
        while sessions and total > self.budget:
            d = sessions.pop(0); total -= sizes[d]
            shutil.rmtree(os.path.join(base, d), ignore_errors=True)
        return total

    def trim_ros_logs(self):
        """ROS node logs (output="log") grow without limit: keep the last half cap of any file over the cap."""
        for root, _, files in os.walk(os.path.join(self.log_dir, 'ros')):
            for f in files:
                p = os.path.join(root, f)
                try:
                    if os.path.getsize(p) > self.cap:
                        with open(p, 'rb') as fh:
                            fh.seek(-self.cap // 2, os.SEEK_END); keep = fh.read()
                        with open(p, 'wb') as fh:
                            fh.write(b'[... trimmed by mission control ...]\n' + keep)
                except OSError:
                    pass

    def ssh(self, remote_cmd, timeout=10):
        host = self.cfg['drone']['host']
        args = ['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=5', '-o', 'ControlMaster=auto',
                '-o', 'ControlPath=' + self.ctl_path, '-o', 'ControlPersist=600', '-o', 'ServerAliveInterval=3',
                host, remote_cmd]
        try:
            r = subprocess.run(args, capture_output=True, text=True, timeout=timeout)
            return r.returncode, r.stdout, r.stderr.strip()
        except subprocess.TimeoutExpired:
            return 255, '', 'timed out'

    def command(self, task):
        """The shell command for the task's chosen variant (recording builds its rosbag command)."""
        v = task.variants()[task.variant]
        if not task.spec.get('record'):
            return v.get('cmd', '').format(**task.choice) if task.choice else v.get('cmd', '')
        if task.remote:
            out = self.cfg['drone'].get('bag_dir', '~/bags')
            prefix = 'mkdir -p %s && ' % out
        else:
            out, prefix = self.bag_dir, ''
        topics = ' '.join(shlex.quote(t) for t in v['topics'])
        name = task.group.name + '_' + task.id.replace('record_', '')            # e.g. sim_mission_<date>.bag
        return prefix + 'rosbag record --lz4 --split --size=%d -o %s/%s %s' % (   # rosbag adds the date
            self.split_mb, out, name, topics)

    # ------------------------------------------------------------------ start / stop
    def worlds(self):
        return assets.worlds(self.worlds_dir, self.cfg['sim'].get('spawn', {}))

    def drones(self):
        return assets.drones(self.urdf_dir, self.cfg['sim'].get('drones', {}), self.cfg['sim'].get('xargs', {}))

    def choose(self, task, world, drone, gazebo=False, planner=None, xargs=None):
        """world/drone for the simulator command, checked against what is on disk. xargs: {name: value} for the drone's
        settings (assets.drones: the config's sim.xargs its URDF declares), each within its range; the rest keep the
        URDF defaults. They go to the launch as {xacro_args}, and the LiDAR tilt also as {pitch}."""
        W = {w['file']: w for w in self.worlds()}
        if world not in W:
            return 'Pick a world'
        D = {d['file']: d for d in self.drones()}
        if drone not in D:
            return 'Pick a drone'
        spec = D[drone]['xargs']
        values = {k: s['default'] for k, s in spec.items()}
        if not isinstance(xargs or {}, dict):
            return 'Drone settings must be name: value pairs'
        for k, v in (xargs or {}).items():
            if k not in spec:
                return 'This drone has no setting %s' % k
            s = spec[k]
            try:
                v = float(v)
            except (TypeError, ValueError):
                return '%s must be a number' % s['label']
            if not s['min'] <= v <= s['max']:
                return '%s must be between %g and %g' % (s['label'], s['min'], s['max'])
            values[k] = v
        omni = any(s['kind'] == '360 depth (ideal)' for s in D[drone].get('sensors', []))
        x, y, z = W[world]['start']; yaw = 0.0
        if planner is not None:                                       # a planner that starts the drone elsewhere (ARiADNE: the pit floor)
            mv = self.groups['sim'].tasks['mission'].variants()
            sp = (mv[int(planner)].get('spawn') or {}).get(world.split('/')[0]) if 0 <= int(planner) < len(mv) else None
            if sp: x, y, z, yaw = sp
        pitch = values.get('lidar_pitch_deg', 23.5)              # setupTF and the planners take it as a launch arg
        task.choice = {'world': shlex.quote(world), 'drone': shlex.quote(drone), 'x': x, 'y': y, 'z': z,
                       'gui': 'true' if gazebo else 'false', 'pitch': pitch, 'yaw': yaw, 'world_file': world, 'drone_file': drone,
                       'xargs': values, 'xacro_args': shlex.quote(' '.join('%s=%g' % kv for kv in sorted(values.items()))),
                       'omni': 'true' if omni else 'false'}
        return None

    def planner_choice(self, task, group, variant, settings):
        """Mission: the map config for the running world (exploration) and an overrides file from the settings."""
        v = task.variants()[variant]
        if v.get('manual'):
            return 'Manual flight needs no planner'
        sim = self.groups[group].tasks.get('sim')
        world = sim.choice.get('world_file') if sim and sim.state in ('running', 'starting') else None
        pitch = sim.choice.get('pitch') if sim and sim.state in ('running', 'starting') else None
        choice = {}
        if v.get('own_sim'):                                         # e.g. EPIC on MARSIM: the world comes from the page
            world = settings.pop('_world', None) if settings else None
        if not self.requirement_met(v, world):
            return '%s has no setup for this world yet' % v['label']
        if '{map}' in v.get('cmd', ''):
            choice['map'] = world.split('/')[0]
        choice['pitch'] = 23.5 if pitch is None else pitch
        if v.get('setup'):
            choice['setup_path'] = expand(v['setup'])
        values = {}
        for f in self.cfg.get('settings', []):
            k = f['key']
            if k.startswith('_') or k not in (settings or {}):
                continue
            try:
                values[k] = bool(settings[k]) if f.get('type') == 'bool' else float(settings[k])
            except (TypeError, ValueError):
                return 'Bad value for ' + f['label']
        if pitch is not None and 'exploration' in v.get('dir', ''):     # the planner's own model of the LiDAR follows the tilt
            values.update({'DEP/lidar_pitch': pitch, 'DEP/max_edge_down_angle': 7.0 + pitch, 'DEP/max_edge_up_angle': 52.0 - pitch})
        if '{overrides}' not in v.get('cmd', ''):
            task.choice = choice; return None
        path = os.path.join(self.log_dir, 'overrides_%s.yaml' % group)
        with open(path, 'w') as fh:
            fh.write('# written by Mission Control for this launch: values changed in the window\n')
            for k, val in values.items():
                fh.write('%s: %s\n' % (k, json.dumps(val)))
        choice['overrides'] = shlex.quote(path)
        task.choice = choice
        return None

    def requirement_met(self, v, world):
        """A planner variant's 'requires' file exists for this world (e.g. its map config)."""
        req = v.get('requires')
        if not req: return True
        if not world: return False
        path = req.format(planner_cfg=self.cfg['sim'].get('planner_cfg', ''), world=world.split('/')[0])
        return os.path.isfile(expand(path))

    def planners_for(self, world):
        return [self.requirement_met(v, world) for v in self.groups['sim'].tasks['mission'].variants()]

    def settings(self, mode_dir):
        """The settings form: each field with its value in that mode's config files (and the map's, if it sets it)."""
        import yaml
        base = os.path.join(expand(self.cfg['sim'].get('planner_cfg', '')), mode_dir)
        out, cache = [], {}
        for f in self.cfg.get('settings', []):
            item = dict(f)
            if f.get('type') != 'info':
                fp = os.path.join(base, f['file'])
                if not os.path.isfile(fp):
                    continue
                if fp not in cache:
                    try: cache[fp] = yaml.safe_load(open(fp)) or {}
                    except Exception: cache[fp] = {}
                item['value'] = cache[fp].get(f['path'])
                if item['value'] is None:
                    continue
            out.append(item)
        return out

    # ------------------------------------------------------------------ flight history
    def history_dirs(self):
        return self.cfg.get('history', {}).get('dirs', ['~/mission_runs'])

    def run_path(self, rid):
        """The file of a run id (a path under home), only inside the history folders."""
        p = os.path.realpath(os.path.join(os.path.expanduser('~'), rid))
        ok = any(p.startswith(os.path.realpath(expand(d)) + os.sep) for d in self.history_dirs())
        return p if ok and os.path.isfile(p) else None

    def runs(self):
        maps = sorted({w['group'] for w in self.worlds()})
        out = history.runs(self.history_dirs(), maps)
        for r in out:
            r['bags'] = history.bags_for(r, [self.bag_dir])
        return out

    def play_run(self, rid):
        r = next((x for x in self.runs() if x['id'] == rid), None)
        if not r or not r['bags']: return 'No recording for this run'
        t = self.groups['sim'].tasks['playback']
        if t.state in ('running', 'starting'): self.stop('sim', 'playback'); time.sleep(1)
        t.choice = {'bags': ' '.join(shlex.quote(b['path']) for b in r['bags'])}
        return self.start('sim', 'playback')

    def save_run(self, d):
        """A session with no planner log (manual flight): its summary and track, for the history."""
        out = expand(os.path.join(self.root, 'runs')); os.makedirs(out, exist_ok=True)
        stamp = time.strftime('%Y%m%d_%H%M%S', time.localtime(d.get('start') or time.time()))
        json.dump(d, open(os.path.join(out, stamp + '_run.json'), 'w'))
        return 'ok'

    def save_settings(self, mode_dir, values):
        """Write changed flight settings into the planner's config files, keeping each line's comment."""
        base = os.path.realpath(os.path.join(expand(self.cfg['sim'].get('planner_cfg', '')), mode_dir))
        if not mode_dir or not base.startswith(os.path.realpath(expand(self.cfg['sim'].get('planner_cfg', '')))):
            return 'unknown planner config'
        fields = {f['key']: f for f in self.cfg.get('settings', []) if f.get('type') != 'info'}
        done = []
        for k, val in values.items():
            f = fields.get(k)
            if not f: continue
            fp = os.path.join(base, f['file'])
            if not os.path.isfile(fp): continue
            text = open(fp).read()
            lit = ('true' if val else 'false') if f.get('type') == 'bool' else ('%g' % float(val))
            pat = re.compile(r'^(%s:[ \t]*)([^#\n]*?)([ \t]*#.*)?$' % re.escape(f['path']), re.M)
            if not pat.search(text): continue
            text = pat.sub(lambda m: m.group(1) + lit + (m.group(3) or ''), text, count=1)
            open(fp, 'w').write(text); done.append(f['label'])
        return 'ok' if done else 'nothing saved'

    def start(self, group, tid, variant=None, world=None, drone=None, gazebo=False, settings=None, planner=None, xargs=None):
        task = self.groups[group].tasks[tid]
        if tid == 'mission' and task.state not in ('running', 'starting', 'stopping'):
            vi = max(0, min(int(variant or 0), len(task.variants()) - 1))
            err = self.planner_choice(task, group, vi, settings)
            if err: return err
        if task.spec.get('choose') and task.state not in ('running', 'starting', 'stopping'):
            err = self.choose(task, world, drone, gazebo, planner, xargs)
            if err: return err
        with task.lock:
            if task.state in ('running', 'starting', 'stopping'):
                return 'already ' + task.state
            if variant is not None:
                task.variant = max(0, min(int(variant), len(task.variants()) - 1))
            if not task.configured():
                return 'This button is not set up yet (config.json)'
            needs = task.variants()[task.variant].get('needs', task.spec.get('needs', [])) if variant is not None or task.spec.get('variants') else task.spec.get('needs', [])
            missing = [n for n in needs if self.groups[group].tasks[n].state != 'running']
            if missing:
                return 'Start first: ' + ', '.join(self.groups[group].tasks[n].spec['label'] for n in missing)
            if task.spec.get('record') and not task.remote and self.free_bytes() < self.min_free + self.split_mb * MB:
                return 'Not enough disk space to record (%.1f GB free)' % (self.free_bytes() / GB)
            if task.remote and not self.drone['connected']:
                return 'Connect to the drone first'
            cmd = self.command(task)
            task.state, task.code, task.started, task.message = 'starting', None, time.time(), ''
            log = self.log(task)
            log.write('\n=== %s  %s\n' % (time.strftime('%H:%M:%S'), cmd))
        if task.remote:
            threading.Thread(target=self._start_remote, args=(task, cmd), daemon=True).start()
        else:
            if tid != 'core' and not self.ensure_master(group):
                task.state, task.message = 'failed', 'ROS master did not come up (see the ROS master log)'
                return task.message
            self._start_local(task, cmd)
        for other in self.groups[group].tasks.values():   # e.g. the live view comes up with the first local task
            if other.spec.get('auto') and other is not task and other.state in ('stopped', 'failed') and other.remote == task.remote:
                self.start(group, other.id)
        return 'ok'

    def master_up(self, group):
        uri = urlparse(self.groups[group].cfg.get('env', {}).get('ROS_MASTER_URI') or os.environ.get('ROS_MASTER_URI', 'http://localhost:11311'))
        try:
            socket.create_connection((uri.hostname, uri.port or 11311), timeout=0.5).close(); return True
        except OSError:
            return False

    def ensure_master(self, group):
        """One ROS master per tab, up before any launch: two roslaunch starting their own master at once collide
        ('run_id on parameter server does not match')."""
        core = self.groups[group].tasks.get('core')
        if core is None or self.master_up(group):
            return True
        if core.state not in ('running', 'starting'):
            self.start(group, 'core')
        for _ in range(60):
            if self.master_up(group): return True
            time.sleep(0.25)
        return False

    def _start_local(self, task, cmd):
        setup = task.choice.get('setup_path') or expand(self.groups[task.group.name].cfg['setup'])
        env = dict(os.environ, ROS_LOG_DIR=os.path.join(self.log_dir, 'ros'), PYTHONUNBUFFERED='1')
        for k, v in self.groups[task.group.name].cfg.get('env', {}).items():   # "" removes the variable
            if v == '': env.pop(k, None)
            else: env[k] = os.path.expanduser(v)
        try:
            p = subprocess.Popen(['bash', '-c', 'source %s && %s' % (shlex.quote(setup), cmd)], stdout=subprocess.PIPE,
                                 stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL, start_new_session=True, env=env,
                                 text=True, errors='replace', bufsize=1)
        except OSError as e:
            task.state, task.message = 'failed', str(e); return
        self.procs[task.key] = p
        task.state = 'running'

        def pump():
            for line in p.stdout:
                self.logs[task.key].write(line)
            code = p.wait()
            with task.lock:
                if task.state == 'stopping' or code in (0, -2, 130):
                    task.state = 'stopped'
                else:
                    task.state, task.message = 'failed', 'exited with code %s' % code
                    self.notice('error', '%s stopped unexpectedly (code %s)' % (task.spec['label'], code))
                task.code = code
            self.logs[task.key].write('=== exited (%s)\n' % code)
        threading.Thread(target=pump, daemon=True).start()

    def _session(self, task):
        return 'mc_' + task.id

    def _start_remote(self, task, cmd):
        setup = self.cfg['drone']['setup']
        name = self._session(task)
        logf = '~/.mission_control/%s.log' % task.id
        inner = 'source %s && %s 2>&1 | tee %s' % (setup, cmd, logf)
        rc, _, err = self.ssh('mkdir -p ~/.mission_control && tmux new-session -d -s %s %s'
                              % (name, shlex.quote('bash -c ' + shlex.quote(inner))))
        with task.lock:
            if rc == 0:
                task.state = 'running'
            else:
                task.state, task.message = 'failed', err or 'could not start on the drone'
                self.log(task).write('=== start failed: %s\n' % err)

    def stop(self, group, tid):
        task = self.groups[group].tasks[tid]
        with task.lock:
            if task.state not in ('running', 'starting'):
                task.state = 'stopped' if task.state == 'failed' else task.state
                return 'ok'
            task.state = 'stopping'
        target = self._stop_remote if task.remote else self._stop_local
        threading.Thread(target=target, args=(task,), daemon=True).start()
        return 'ok'

    def kill(self, group, tid):
        """Force quit: no clean shutdown, everything in the task's process group (or tmux session) dies now."""
        task = self.groups[group].tasks[tid]
        with task.lock:
            if task.state not in ('running', 'starting', 'stopping'):
                return 'ok'
            task.state = 'stopping'
        self.log(task).write('=== force quit\n')
        if task.remote:
            self.ssh('tmux kill-session -t %s' % self._session(task)); task.state = 'stopped'
        else:
            p = self.procs.get(task.key)
            if p is not None and p.poll() is None:
                try: os.killpg(p.pid, signal.SIGKILL)
                except ProcessLookupError: pass
        return 'ok'

    # ------------------------------------------------------------------ debug terminals
    def open_terminal(self, group):
        g = self.groups[group]
        if group == 'drone' and not self.cfg['drone'].get('local_shell'):
            host = self.cfg['drone']['host']
            argv = ['ssh', '-t', '-o', 'ControlMaster=auto', '-o', 'ControlPath=' + self.ctl_path, '-o', 'ControlPersist=600',
                    host, 'bash -lc %s' % shlex.quote('source %s; exec bash -i' % g.cfg['setup'])]
            title = 'drone shell'
        else:
            rc = os.path.join(self.log_dir, 'term_%s.rc' % group)
            lines = ['[ -f ~/.bashrc ] && source ~/.bashrc', 'source %s' % shlex.quote(expand(g.cfg['setup']))]
            for k, v in g.cfg.get('env', {}).items():         # after .bashrc, which may point ROS elsewhere
                lines.append('unset %s' % k if v == '' else 'export %s=%s' % (k, shlex.quote(os.path.expanduser(v))))
            lines.append('export PS1="\\[\\e[38;5;245m\\]%s \\[\\e[0m\\]\\W \\$ "' % group)
            lines.append('echo "ROS_MASTER_URI=$ROS_MASTER_URI   try: rostopic list, rosnode list, rosnode info <node>"')
            open(rc, 'w').write('\n'.join(lines) + '\n')
            argv, title = ['bash', '--rcfile', rc, '-i'], group + ' shell'
        env = dict(os.environ, TERM='xterm-256color')
        return self.terms.open(argv, env, title, cwd=os.path.expanduser('~'))

    def _stop_local(self, task):
        p = self.procs.get(task.key)
        if p is None or p.poll() is not None:
            task.state = 'stopped'; return
        for sig, wait in ((signal.SIGINT, 15), (signal.SIGTERM, 5), (signal.SIGKILL, 5)):
            try: os.killpg(p.pid, sig)
            except ProcessLookupError: break
            try:
                p.wait(timeout=wait); break
            except subprocess.TimeoutExpired:
                self.logs[task.key].write('=== still running %d s after %s\n' % (wait, sig.name))

    def _stop_remote(self, task):
        name = self._session(task)
        self.ssh('tmux send-keys -t %s C-c' % name)
        for _ in range(15):
            rc, _, _ = self.ssh('tmux has-session -t %s' % name)
            if rc == 1: break
            time.sleep(1)
        else:
            self.ssh('tmux kill-session -t %s' % name)
        task.state = 'stopped'

    def stop_all(self):
        self.terms.close_all()
        for g in self.groups.values():
            for t in g.tasks.values():
                if not t.remote and t.state in ('running', 'starting'):
                    self.stop(g.name, t.id)

    # ------------------------------------------------------------------ drone link
    def connect(self):
        rc, out, err = self.ssh('command -v tmux >/dev/null && echo tmux-ok; tmux ls -F "#S" 2>/dev/null', timeout=12)
        d = self.drone
        d['checked'] = time.time()
        if rc != 0:
            d['connected'], d['message'] = False, 'Cannot reach the drone: ' + (err or 'ssh failed') + \
                ('' if 'Permission' not in err else ' (set up key login with ssh-copy-id)')
        elif 'tmux-ok' not in out:
            d['connected'], d['message'] = False, 'tmux is not installed on the drone (sudo apt install tmux)'
        else:
            d['connected'], d['message'] = True, 'Connected'
            self._adopt(out.split())
        return d['message']

    def disconnect(self):
        subprocess.run(['ssh', '-o', 'ControlPath=' + self.ctl_path, '-O', 'exit', self.cfg['drone']['host']],
                       capture_output=True)
        self.drone.update(connected=False, message='Disconnected (drone tasks keep running)')

    def _adopt(self, sessions):
        """Show tasks already running on the drone (started earlier, or before a WiFi drop) as running."""
        self.drone['sessions'] = sessions
        for t in self.groups['drone'].tasks.values():
            if not t.remote: continue
            live = self._session(t) in sessions
            if live and t.state in ('stopped', 'failed'):
                t.state, t.started = 'running', t.started or time.time()
            elif not live and t.state == 'running':
                t.state = 'stopped'

    def remote_log(self, task, n):
        rc, out, err = self.ssh('tail -n %d ~/.mission_control/%s.log 2>/dev/null' % (n, task.id))
        return out.splitlines() if rc == 0 else ['(could not read the drone log: %s)' % err]

    # ------------------------------------------------------------------ background watch
    def watch(self):
        n = 0
        while True:
            time.sleep(2); n += 1
            try:
                free = self.free_bytes()
                if free < self.min_free:
                    for g in self.groups.values():
                        for t in g.tasks.values():
                            if t.spec.get('record') and not t.remote and t.state == 'running':
                                self.notice('error', '%s stopped: disk almost full (%.1f GB free)' % (t.spec['label'], free / GB))
                                self.stop(g.name, t.id)
                now, b = time.time(), self.bag_bytes()
                t0, b0 = self._last_bag
                self.bag_rate = max(0.0, (b - b0) / max(now - t0, 1e-3)); self._last_bag = (now, b)
                if self.drone['connected']:
                    bag_dir = self.cfg['drone'].get('bag_dir', '~/bags')
                    rc, out, err = self.ssh('tmux ls -F "#S" 2>/dev/null; echo ---; df -B1 --output=avail ~ | tail -1; '
                                            'du -sb %s 2>/dev/null | cut -f1' % bag_dir, timeout=6)
                    if rc == 255:
                        self.drone.update(connected=False, message='Link lost (drone tasks keep running). Reconnect.')
                        self.notice('error', 'Lost the link to the drone')
                    else:
                        head, _, tail = out.partition('---')
                        self._adopt(head.split())
                        nums = tail.split()
                        self.drone['free_gb'] = int(nums[0]) / GB if nums else None
                        self.drone['bags_gb'] = int(nums[1]) / GB if len(nums) > 1 else 0.0
                for g in self.groups.values():                     # a recording outliving its session would fill the disk
                    for t in g.tasks.values():
                        if t.spec.get('record') and not t.remote and t.state == 'running' and time.time() - (t.started or 0) > 10 \
                                and not any(o.state in ('running', 'starting') for k, o in g.tasks.items() if k in ('sim', 'mission')):
                            self.notice('error', '%s stopped: the session it was recording has ended' % t.spec['label'])
                            self.stop(g.name, t.id)
                if n % 15 == 0:
                    self.trim_ros_logs()
                if n % 30 == 0:
                    self.prune_logs()
            except Exception as e:  # keep watching whatever happens
                self.notice('error', 'monitor: %s' % e)

    # ------------------------------------------------------------------ state for the page
    def state(self):
        free = self.free_bytes()
        rate = self.bag_rate
        return {
            'session': self.session,
            'groups': {k: {'label': g.cfg['label'], 'tasks': [t.info() for t in g.tasks.values()], 'viz': g.cfg['viz'],
                           'default_world': g.cfg.get('default_world'), 'default_drone': g.cfg.get('default_drone')}
                       for k, g in self.groups.items()},
            'drone': dict(self.drone, host=self.cfg['drone']['host']),
            'disk': {'free_gb': free / GB, 'total_gb': shutil.disk_usage(self.root).total / GB,
                     'min_free_gb': self.min_free / GB, 'bags_gb': self._last_bag[1] / GB,
                     'logs_mb': dir_size(os.path.join(self.root, 'logs')) / MB, 'log_budget_mb': self.budget / MB,
                     'bag_rate_mbs': rate / MB,
                     'record_left_min': (free - self.min_free) / rate / 60 if rate > 1e4 else None},
            'notices': list(self.notices),
        }

    def bags(self):
        out = []
        for f in sorted(os.listdir(self.bag_dir), reverse=True):
            p = os.path.join(self.bag_dir, f)
            if os.path.isfile(p) and (f.endswith('.bag') or f.endswith('.bag.active')):
                out.append({'name': f, 'mb': os.path.getsize(p) / MB, 'mtime': os.path.getmtime(p)})
        return out

    def delete_bag(self, name):
        p = os.path.realpath(os.path.join(self.bag_dir, name))
        if os.path.dirname(p) != os.path.realpath(self.bag_dir) or not p.endswith('.bag'):
            return 'refused'
        os.remove(p)
        return 'ok'


def make_handler(app):
    web = os.path.join(HERE, 'web')
    types = {'.html': 'text/html', '.js': 'application/javascript', '.css': 'text/css', '.svg': 'image/svg+xml'}

    class H(BaseHTTPRequestHandler):
        def log_message(self, *a): pass

        def send(self, code, body, ctype='application/json'):
            data = body if isinstance(body, bytes) else json.dumps(body).encode()
            self.send_response(code)
            self.send_header('Content-Type', ctype)
            self.send_header('Content-Length', str(len(data)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            u = urlparse(self.path); q = parse_qs(u.query); parts = u.path.strip('/').split('/')
            if u.path == '/api/state':
                return self.send(200, app.state())
            if u.path == '/api/events':     # server-sent events: the state every second
                self.send_response(200)
                self.send_header('Content-Type', 'text/event-stream'); self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                try:
                    while True:
                        self.wfile.write(b'data: ' + json.dumps(app.state()).encode() + b'\n\n'); self.wfile.flush()
                        time.sleep(1)
                except (BrokenPipeError, ConnectionResetError):
                    return
            if parts[:2] == ['api', 'log'] and len(parts) == 4:
                task = app.groups[parts[2]].tasks[parts[3]]
                n = int(q.get('n', ['300'])[0])
                lines = app.remote_log(task, n) if task.remote else (app.logs[task.key].tail(n) if task.key in app.logs else [])
                return self.send(200, {'lines': lines})
            if u.path == '/api/bags':
                return self.send(200, {'bags': app.bags(), 'dir': app.bag_dir})
            if u.path == '/api/worlds':
                W = app.worlds(); notes = app.cfg['sim'].get('world_notes', {})
                for w in W:
                    w['planners'] = app.planners_for(w['file']); w['notes'] = notes.get(w['group'])
                return self.send(200, {'worlds': W})
            if u.path == '/api/runs':
                return self.send(200, {'runs': app.runs()})
            if u.path in ('/api/runs/track', '/api/runs/svg'):
                p = app.run_path(q.get('id', [''])[0])
                if not p: return self.send(404, {'message': 'no such run'})
                if u.path.endswith('svg'):
                    sv = p[:-4] + '.svg'
                    return self.send(200, open(sv, 'rb').read(), 'image/svg+xml') if os.path.isfile(sv) else self.send(404, {'message': 'no picture'})
                return self.send(200, {'track': history.track(p)})
            if u.path == '/api/settings':
                return self.send(200, {'fields': app.settings(q.get('dir', ['dynamic_exploration'])[0])})
            if u.path == '/api/drones':
                return self.send(200, {'drones': app.drones()})
            if u.path == '/api/terms':
                return self.send(200, {'terms': app.terms.list()})
            if parts[:2] == ['api', 'term'] and len(parts) == 4 and parts[3] == 'stream':
                t = app.terms.get(parts[2])
                if t is None: return self.send(404, {'message': 'no such terminal'})
                self.send_response(200)
                self.send_header('Content-Type', 'text/event-stream'); self.send_header('Cache-Control', 'no-store')
                self.end_headers()

                def send(chunk):
                    self.wfile.write(b'data: ' + chunk.encode() + b'\n\n'); self.wfile.flush()
                try:
                    t.stream(send)
                    self.wfile.write(b'event: exit\ndata: \n\n'); self.wfile.flush()
                except (BrokenPipeError, ConnectionResetError):
                    pass
                return
            if parts[0] == 'models' and len(parts) >= 3:           # CAD drone meshes: urdf/<model>/(model.json|meshes/*.stl)
                base = os.path.realpath(app.urdf_dir)
                p = os.path.realpath(os.path.join(base, *parts[1:]))
                if p.startswith(base + os.sep) and os.path.isfile(p) and p.endswith(('.stl', '.json')):
                    return self.send(200, open(p, 'rb').read(), 'model/stl' if p.endswith('.stl') else 'application/json')
                return self.send(404, {'error': 'not found'})
            path = os.path.realpath(os.path.join(web, u.path.lstrip('/') or 'index.html'))
            if not path.startswith(os.path.realpath(web)) or not os.path.isfile(path):
                return self.send(404, {'error': 'not found'})
            self.send(200, open(path, 'rb').read(), types.get(os.path.splitext(path)[1], 'application/octet-stream'))

        def do_POST(self):
            parts = urlparse(self.path).path.strip('/').split('/')
            n = int(self.headers.get('Content-Length') or 0)
            body = json.loads(self.rfile.read(n) or b'{}') if n else {}
            try:
                if parts[:2] == ['api', 'task'] and len(parts) == 5:
                    _, _, group, tid, action = parts
                    if action == 'start':
                        msg = app.start(group, tid, body.get('variant'), body.get('world'), body.get('drone'),
                                        bool(body.get('gazebo')), body.get('settings'), body.get('planner'), body.get('xargs'))
                    elif action == 'kill':
                        msg = app.kill(group, tid)
                    else:
                        msg = app.stop(group, tid)
                    return self.send(200, {'message': msg})
                if parts == ['api', 'runs', 'play']:
                    return self.send(200, {'message': app.play_run(body.get('id', ''))})
                if parts == ['api', 'runs', 'save']:
                    return self.send(200, {'message': app.save_run(body)})
                if parts == ['api', 'settings', 'save']:
                    return self.send(200, {'message': app.save_settings(body.get('dir', ''), body.get('values', {}))})
                if parts == ['api', 'term', 'open']:
                    t = app.open_terminal(body.get('group', 'sim'))
                    return self.send(200, {'id': t.id, 'title': t.title})
                if parts[:2] == ['api', 'term'] and len(parts) == 4:
                    t = app.terms.get(parts[2])
                    if t is None: return self.send(404, {'message': 'no such terminal'})
                    if parts[3] == 'input':
                        t.write(base64.b64decode(body.get('data', ''))); return self.send(200, {'message': 'ok'})
                    if parts[3] == 'resize':
                        t.resize(int(body['cols']), int(body['rows'])); return self.send(200, {'message': 'ok'})
                    if parts[3] == 'close':
                        app.terms.close(parts[2]); return self.send(200, {'message': 'ok'})
                if parts == ['api', 'drone', 'connect']:
                    return self.send(200, {'message': app.connect()})
                if parts == ['api', 'drone', 'disconnect']:
                    app.disconnect(); return self.send(200, {'message': 'ok'})
                if parts == ['api', 'bags', 'delete']:
                    return self.send(200, {'message': app.delete_bag(body.get('name', ''))})
                if parts == ['api', 'logs', 'prune']:
                    return self.send(200, {'message': 'Logs now %.0f MB' % (app.prune_logs() / MB)})
            except KeyError as e:
                return self.send(404, {'message': 'unknown %s' % e})
            self.send(404, {'message': 'unknown request'})
    return H


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--port', type=int, default=8765)
    ap.add_argument('--config', default=os.path.join(HERE, 'config.json'))
    a = ap.parse_args()
    app = App(a.config)
    srv = ThreadingHTTPServer(('127.0.0.1', a.port), make_handler(app))
    srv.daemon_threads = True

    def bye(*_):
        app.stop_all()
        deadline = time.time() + 25     # let launches shut down cleanly before we exit
        while time.time() < deadline and any(t.state in ('running', 'stopping') and not t.remote
                                             for g in app.groups.values() for t in g.tasks.values()):
            time.sleep(0.5)
        os._exit(0)
    signal.signal(signal.SIGINT, bye); signal.signal(signal.SIGTERM, bye)
    print('CERLAB UAV Mission Control on http://127.0.0.1:%d  (logs and bags in %s)' % (a.port, app.root), flush=True)
    srv.serve_forever()


if __name__ == '__main__':
    main()
