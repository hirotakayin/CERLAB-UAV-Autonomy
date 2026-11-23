"""Flight history: every run the planners saved, in one list.

The mission monitor writes each run to ~/mission_runs (or a batch's run_save_dir under it) as
<YYYYmmdd_HHMMSS>_<preset>_run<N>.csv (time, position, coverage and the plans) plus a .svg of the path; batch folders
add run_info.json (map, preset). Mission Control's own sessions with no planner (manual flights) are saved as
<root>/runs/<stamp>.json. This module lists them all with what a history screen needs (date, map, planner, duration,
coverage, path length), returns a run's track for replay, and finds the recordings (bags) made during it.
Results are cached by file modification time.
"""
import csv
import glob
import json
import math
import os
import re
import time

_cache = {}
STAMP = re.compile(r'(\d{8}_\d{6})_(.*)_run(\d+)$')


def planner_of(preset):
    p = preset.lower()
    if 'epic' in p: return 'EPIC'
    if 'ariadne' in p: return 'ARiADNE'
    if 'baseline' in p: return 'HIRE'
    if 'manual' in p: return 'Manual'
    return 'HIRE 3D'


def map_of(preset, info, maps):
    if info.get('map'): return info['map']
    for m in sorted(maps, key=len, reverse=True):                 # the longest map name the preset starts with
        if preset.startswith(m + '_') or preset == m: return m
    return 'excavation1'                                          # presets without a map prefix ran in excavation1


def read_csv(path):
    rows = []
    with open(path, newline='') as fh:
        for r in csv.DictReader(fh):
            try:
                rows.append((float(r['t_s']), float(r['x_m']), float(r['y_m']), float(r['z_m']), r.get('state', ''),
                             float(r['pit_block_pct'] or 'nan'), float(r.get('floor_pct') or 'nan'), float(r.get('surface_pct') or 'nan')))
            except (KeyError, ValueError):
                continue
    return rows


def summarise_csv(path, maps):
    m = STAMP.match(os.path.splitext(os.path.basename(path))[0])
    if not m: return None
    stamp, preset, n = m.groups()
    info_p = os.path.join(os.path.dirname(path), 'run_info.json')
    info = json.load(open(info_p)) if os.path.isfile(info_p) else {}
    rows = read_csv(path)
    if len(rows) < 2: return None
    length = sum(math.dist(a[1:4], b[1:4]) for a, b in zip(rows, rows[1:]))
    last = lambda k: next((r[k] for r in reversed(rows) if r[k] == r[k]), None)
    start = time.mktime(time.strptime(stamp, '%Y%m%d_%H%M%S'))
    return {'start': start, 'duration': rows[-1][0] - rows[0][0], 'preset': info.get('preset', preset), 'map': map_of(preset, info, maps),
            'planner': planner_of(info.get('preset', preset)), 'run': int(n), 'path_m': round(length, 1),
            'explored': last(5), 'floor': last(6), 'surface': last(7), 'end_state': rows[-1][4],
            'min_z': round(min(r[3] for r in rows), 2), 'max_z': round(max(r[3] for r in rows), 2),
            'static': '_static' in preset, 'has_svg': os.path.isfile(path[:-4] + '.svg')}


def summarise_json(path):
    d = json.load(open(path))
    return {k: d.get(k) for k in ('start', 'duration', 'preset', 'map', 'planner', 'path_m', 'explored', 'floor', 'surface',
                                  'end_state', 'min_z', 'max_z', 'world', 'drone', 'pitch')} | {'run': 1, 'has_svg': False}


def runs(dirs, maps):
    out = []
    for base in dirs:
        base = os.path.expanduser(base)
        for path in glob.glob(os.path.join(base, '**', '*_run*.csv'), recursive=True) + glob.glob(os.path.join(base, '*.json')):
            try:
                key = (path, os.path.getmtime(path))
                if key not in _cache:
                    _cache[key] = summarise_json(path) if path.endswith('.json') else summarise_csv(path, maps)
                s = _cache[key]
            except Exception:
                continue
            if not s: continue
            rel = os.path.relpath(path, os.path.expanduser('~'))
            out.append(dict(s, id=rel, folder=os.path.dirname('~/' + rel), file=os.path.basename(path)))
    out.sort(key=lambda r: r['start'] or 0, reverse=True)
    return out


def track(path, max_points=3000):
    """[[t, x, y, z], ...] of a run, thinned to max_points."""
    if path.endswith('.json'):
        pts = json.load(open(path)).get('track', [])
    else:
        pts = [[r[0], r[1], r[2], r[3]] for r in read_csv(path)]
    k = max(1, len(pts) // max_points)
    return pts[::k] + ([pts[-1]] if pts and (len(pts) - 1) % k else [])


BAG_STAMP = re.compile(r'(\d{4}-\d{2}-\d{2}-\d{2}-\d{2}-\d{2})')


def bags_for(run, bag_dirs, slack=60):
    """Recordings whose start falls inside the run (plus a minute either side)."""
    t0, t1 = run['start'] - slack, run['start'] + (run['duration'] or 0) + slack
    found = []
    for d in bag_dirs:
        for p in glob.glob(os.path.join(os.path.expanduser(d), '*.bag')):
            m = BAG_STAMP.search(os.path.basename(p))
            if not m: continue
            t = time.mktime(time.strptime(m.group(1), '%Y-%m-%d-%H-%M-%S'))
            if t0 <= t <= t1:
                found.append({'path': p, 'name': os.path.basename(p), 'mb': round(os.path.getsize(p) / 2 ** 20, 1)})
    return sorted(found, key=lambda b: b['name'])
