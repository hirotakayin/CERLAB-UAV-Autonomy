#!/usr/bin/env python3
"""Record the mission dashboard's 3D view to video, in hidden browser windows, while you use the screen for other work.

Start it after the planner launch, before or after pressing Start mission:

  rosrun autonomous_flight record_dashboard.py
  rosrun autonomous_flight record_dashboard.py --views paths_voxels --size 1920x1080
  rosrun autonomous_flight record_dashboard.py --views paths --name excavation1_gain_r1 --out ~/mission_runs/x   # run_batch.py

Each view is its own hidden browser showing the dashboard with a fixed set of layers and camera. Recording stops 5 s after
the drone lands, or on Ctrl+C, and every video is converted to MP4 in the output folder. Next to each video, a .json
file holds the wall-clock time of its first frame (video_start_epoch), which run_highlights.py uses to place log events.

Views:
  paths_voxels  map voxels, candidate A* paths, chosen path and goal candidates, from a fixed camera; voxels more than
                1.5 m above the drone are hidden, so slabs, beams and cave roofs never cover it
  paths         no voxels, candidate A* paths and chosen path, from the same fixed camera
  orbit         like paths, with the camera slowly circling the pit
The fixed camera views the whole pit from above at an angle, so the videos work as GIFs.
"""
import argparse
import glob
import http.server
import json
import os
import shutil
import signal
import socketserver
import subprocess
import tempfile
import threading
import time
import urllib.parse

GUI = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'gui')
VIEWS = {
    'paths_voxels': 'layers=goals,candidates,best&cam=overview&cut=1.5',
    'paths': 'layers=candidates,best&voxels=0&cam=overview',
    'orbit': 'layers=candidates,best&voxels=0&cam=orbit',
}
BROWSERS = ['google-chrome', 'google-chrome-stable', 'chromium', 'chromium-browser', '/snap/bin/chromium']


class Recorder(http.server.SimpleHTTPRequestHandler):
    """Serves the dashboard page and takes the video pieces the page sends back."""
    out_dir = ''
    name = '' # with --name, videos are <name>_<view>.mp4
    views = []
    files = {}
    finished = set()
    lock = threading.Lock()

    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=GUI, **kwargs)

    def log_message(self, *args):
        pass

    def do_POST(self):
        url = urllib.parse.urlparse(self.path)
        query = urllib.parse.parse_qs(url.query)
        body = self.rfile.read(int(self.headers.get('Content-Length', 0)))
        if url.path == '/chunk' and 'id' in query:
            page = os.path.basename(query['id'][0])
            with Recorder.lock:
                if page not in Recorder.files:
                    Recorder.files[page] = Recorder.file_for(page)
                    print('Recording ' + Recorder.files[page], flush=True)
                    started = page.rsplit('_', 1)[-1] # the page ends the id with the time it started recording, in ms
                    if started.isdigit():
                        with open(Recorder.files[page][:-5] + '.json', 'w') as f:
                            json.dump({'video_start_epoch': int(started) / 1000.0, 'page': page}, f)
                with open(Recorder.files[page], 'ab') as f:
                    f.write(body)
                if 'done' in query: Recorder.finished.add(page)
        self.send_response(204)
        self.end_headers()

    @staticmethod
    def file_for(page):
        if not Recorder.name:
            return os.path.join(Recorder.out_dir, time.strftime('%Y%m%d_%H%M%S_') + page + '.webm')
        # the id is <run>_<view>_<ms>; the longest view name that fits, so paths_voxels is not taken for paths
        view = max((v for v in Recorder.views if '_' + v + '_' in page), key=len, default='view')
        base = os.path.join(Recorder.out_dir, Recorder.name + '_' + view)
        path, n = base + '.webm', 2
        while path in Recorder.files.values() or os.path.exists(path) or os.path.exists(path[:-5] + '.mp4'):
            path, n = '%s_%d.webm' % (base, n), n + 1 # the page reloaded (rosbridge restarted): a second video
        return path


def find_browser():
    for name in BROWSERS:
        path = shutil.which(name) or (name if os.path.exists(name) else None)
        if path: return path
    raise SystemExit('No Chrome or Chromium found')


def to_mp4(webm):
    mp4 = webm[:-5] + '.mp4'
    # H.264 needs even sides, and the browser's page area can come out odd
    done = subprocess.run(['ffmpeg', '-y', '-loglevel', 'error', '-i', webm, '-vf', 'scale=trunc(iw/2)*2:trunc(ih/2)*2',
                           '-c:v', 'libx264', '-pix_fmt', 'yuv420p', '-r', '25', mp4])
    if done.returncode == 0: return mp4
    if os.path.exists(mp4): os.remove(mp4)
    return None


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--views', nargs='+', default=['paths_voxels', 'paths'], choices=sorted(VIEWS))
    parser.add_argument('--size', default='1600x900', help='video size, WIDTHxHEIGHT')
    parser.add_argument('--port', type=int, default=8765)
    parser.add_argument('--ws', default='ws://localhost:9090', help='rosbridge of the running sim')
    parser.add_argument('--out', default=os.path.expanduser('~/mission_runs/recordings'))
    parser.add_argument('--name', default='', help='name the videos <name>_<view>.mp4 instead of by time and page')
    args = parser.parse_args()
    os.makedirs(args.out, exist_ok=True)
    Recorder.out_dir = args.out
    Recorder.name = args.name
    Recorder.views = args.views

    socketserver.ThreadingTCPServer.allow_reuse_address = True # the last run's server can leave the port in TIME_WAIT
    server = socketserver.ThreadingTCPServer(('127.0.0.1', args.port), Recorder)
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()

    browser = find_browser()
    # a snap Chromium can only keep its profile inside its own folder
    profile_root = os.path.expanduser('~/snap/chromium/common') if browser.startswith('/snap/') else None
    profiles, browsers = [], []
    width, height = args.size.split('x')
    for view in args.views:
        profile = tempfile.mkdtemp(prefix='record_dashboard_', dir=profile_root)
        profiles.append(profile)
        url = 'http://127.0.0.1:%d/mission_control.html?clean=1&ws=%s&record=%s&%s' % (args.port, urllib.parse.quote(args.ws, safe=':/'), view, VIEWS[view])
        browsers.append(subprocess.Popen([browser, '--headless=new', '--user-data-dir=' + profile, '--no-first-run', '--no-default-browser-check',
                                          '--window-size=%s,%s' % (width, height), '--autoplay-policy=no-user-gesture-required',
                                          '--disable-background-timer-throttling', '--disable-renderer-backgrounding', url],
                                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True))
        print('Started view %s' % view, flush=True)
    print('Recording to %s until the drone lands. Ctrl+C to stop early.' % args.out, flush=True)

    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *a: stop.set())
    signal.signal(signal.SIGTERM, lambda *a: stop.set())
    while not stop.is_set():
        with Recorder.lock:
            if Recorder.files and len(Recorder.finished) >= len(args.views): break
        time.sleep(1)
    time.sleep(3) # the last pieces are still on their way
    for b in browsers:
        try: os.killpg(b.pid, signal.SIGTERM)
        except OSError: pass
    server.shutdown()
    for p in profiles: shutil.rmtree(p, ignore_errors=True)
    for webm in sorted(set(Recorder.files.values())):
        print('Converting ' + webm, flush=True)
        mp4 = to_mp4(webm)
        print(('Saved ' + mp4) if mp4 else ('Kept %s (MP4 conversion failed)' % webm), flush=True)
        if mp4: os.remove(webm)
    if not Recorder.files:
        print('Nothing was recorded: is the dashboard\'s rosbridge running (the planner launch with its dashboard)?')


if __name__ == '__main__':
    main()
