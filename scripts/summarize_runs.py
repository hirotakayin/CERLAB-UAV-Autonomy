#!/usr/bin/env python3
"""The numbers for each mission CSV, and the mean and spread for each map and planner.

  rosrun autonomous_flight summarize_runs.py ~/mission_runs/batch_trials       # a folder: every mission CSV in it
  rosrun autonomous_flight summarize_runs.py a.csv b.csv --out summary.csv

Per run, over the exploring phase (end of the entry waypoints to the start of the return):
  explore_s      time spent exploring, in the mission's clock (sim time for Gazebo, real time for EPIC)
  path_m         distance flown while exploring, from the 2 Hz position log (corners cut slightly)
  mapped_pct     pit blocks explored when exploring ended (0.6 m blocks of the scoring box, as on the dashboard)
  voxels_pct     known share of the scoring box's voxels at the same moment
  plan_s         mean computer time to choose a goal: DEP's whole plan, EPIC's global planning step
  min_clear_m    closest the drone came to a mapped obstacle while exploring
mission_s is takeoff to landing, and status says whether it got home.

Each planner stops by its own rule, so explore_s and path_m also measure the rule. These are the same for every run,
taken from the coverage log, with time and distance counted from the start of exploring:
  t90_s, path90_m   time and distance to reach 90% of the best coverage any run reached on the same map
  at120_pct         blocks mapped after 120 s of exploring (blank if the run explored for less)
  plateau_s         when coverage first gained under 1 point over the next 60 s: where a common stop rule would end it
  plateau_pct       blocks mapped at that point
"""
import argparse
import csv
import glob
import math
import os
import re
import statistics

MAPS = ['excavation_long', 'excavation_slab'] # anything else is excavation1


def describe(path):
    name = os.path.basename(path)[:-4]
    m = re.match(r'\d{8}_\d{6}_(.+)_run\d+$', name)
    label = m.group(1) if m else name
    pit = next((p for p in MAPS if label.startswith(p + '_')), 'excavation1')
    planner = label[len(pit) + 1:] if pit != 'excavation1' else label
    return pit, planner


def number(row, key):
    try:
        return float(row[key])
    except (KeyError, ValueError):
        return None


def coverage_series(samples):
    """(time since exploring began, distance flown since then, blocks mapped) through the exploring phase."""
    exploring = [r for r in samples if r['state'] == 'EXPLORING']
    if len(exploring) < 2: return []
    t0, t1 = number(exploring[0], 't_s'), number(exploring[-1], 't_s')
    series, dist, prev = [], 0.0, None
    for r in samples:
        t = number(r, 't_s')
        if t < t0 or t > t1: continue
        p = (number(r, 'x_m'), number(r, 'y_m'), number(r, 'z_m'))
        if prev is not None: dist += math.dist(prev, p)
        prev = p
        series.append((t - t0, dist, number(r, 'pit_block_pct')))
    return series


def add_common_measures(run, series, best):
    if not series: return
    target = 0.9 * best
    hit = next((s for s in series if s[2] >= target), None)
    run['t90_s'], run['path90_m'] = (hit[0], hit[1]) if hit else (None, None)
    at = [s for s in series if s[0] <= 120.0]
    run['at120_pct'] = at[-1][2] if series[-1][0] >= 120.0 and at else None
    for i, (t, d, c) in enumerate(series):
        later = [s for s in series[i:] if s[0] <= t + 60.0]
        if series[-1][0] < t + 60.0: break # not 60 s of log left to judge
        if later[-1][2] - c < 1.0:
            run['plateau_s'], run['plateau_pct'] = t, c
            return
    run['plateau_s'], run['plateau_pct'] = None, None


def summarize(path):
    rows = list(csv.DictReader(open(path)))
    samples = [r for r in rows if r.get('type') == 'height']
    plans = [r for r in rows if r.get('type') == 'waypoint']
    exploring = [r for r in samples if r['state'] == 'EXPLORING']
    pit, planner = describe(path)
    out = {'file': os.path.basename(path), 'map': pit, 'planner': planner}
    if not samples or 'x_m' not in samples[0]:
        out['status'] = 'not a mission log'
        return out
    out['mission_s'] = number(samples[-1], 't_s')
    out['status'] = 'landed' if samples[-1]['state'] in ('LANDING', 'LANDED') and any(r['state'] == 'RETURNING' for r in samples) else 'ended ' + samples[-1]['state'].lower()
    if len(exploring) < 2:
        out['status'] += ', never explored'
        return out
    pos = lambda r: (number(r, 'x_m'), number(r, 'y_m'), number(r, 'z_m'))
    out['explore_s'] = number(exploring[-1], 't_s') - number(exploring[0], 't_s')
    # consecutive exploring samples only: a pause or inspection leg in between is not counted
    path = 0.0
    for a, b in zip(samples, samples[1:]):
        if a['state'] == 'EXPLORING' and b['state'] == 'EXPLORING': path += math.dist(pos(a), pos(b))
    out['path_m'] = path
    out['mapped_pct'] = number(exploring[-1], 'pit_block_pct')
    out['voxels_pct'] = number(exploring[-1], 'pit_pct')
    times = [number(r, 'plan_time_s') for r in plans if number(r, 'plan_time_s') is not None]
    if times and planner.endswith('epic_1ms') and statistics.median(times) > 5.0:
        times = [t / 1000.0 for t in times] # EPIC logs before 9 Nov hold milliseconds in this column
    out['plans'] = len(plans)
    out['plan_s'] = statistics.mean(times) if times else None
    clear = [number(r, 'min_clearance_m') for r in exploring if number(r, 'min_clearance_m') is not None]
    out['min_clear_m'] = min(clear) if clear else None
    out['series'] = coverage_series(samples)
    return out


COLUMNS = ['map', 'planner', 't90_s', 'path90_m', 'at120_pct', 'plateau_s', 'plateau_pct', 'explore_s', 'path_m', 'mapped_pct', 'voxels_pct', 'plan_s', 'min_clear_m', 'plans', 'mission_s', 'status', 'file']
NUMBERS = ['t90_s', 'path90_m', 'at120_pct', 'plateau_s', 'plateau_pct', 'explore_s', 'path_m', 'mapped_pct', 'voxels_pct', 'plan_s', 'min_clear_m']
DECIMALS = {'plan_s': 3, 'min_clear_m': 2}


def fmt(value, key):
    if value is None or value == '': return '-'
    return ('%.' + str(DECIMALS.get(key, 1)) + 'f') % value if isinstance(value, float) else str(value)


def table(rows, columns):
    widths = [max(len(c), *(len(fmt(r.get(c), c)) for r in rows)) for c in columns]
    lines = ['  '.join(c.ljust(w) for c, w in zip(columns, widths))]
    for r in rows: lines.append('  '.join(fmt(r.get(c), c).ljust(w) for c, w in zip(columns, widths)))
    return '\n'.join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('paths', nargs='+', help='mission CSVs, or folders searched for them')
    parser.add_argument('--out', help='also write one row per run to this CSV')
    args = parser.parse_args()
    files = []
    for p in args.paths:
        p = os.path.expanduser(p)
        if os.path.isdir(p): files += sorted(glob.glob(os.path.join(p, '**', '*_run*.csv'), recursive=True))
        elif os.path.isfile(p): files.append(p)
        else: print('Not found: ' + p)
    runs = [summarize(f) for f in files]
    runs.sort(key=lambda r: (r['map'], r['planner'], r['file']))
    best = {}
    for r in runs:
        if r.get('series'): best[r['map']] = max(best.get(r['map'], 0.0), max(s[2] for s in r['series']))
    for r in runs:
        add_common_measures(r, r.get('series'), best.get(r['map'], 0.0))
    print('Best coverage per map, the reference for t90_s: ' + ', '.join('%s %.1f%%' % kv for kv in sorted(best.items())))
    print(table(runs, COLUMNS))

    # mean and spread per map and planner, over runs that explored
    groups = {}
    for r in runs:
        if 'explore_s' in r: groups.setdefault((r['map'], r['planner']), []).append(r)
    summary = []
    for (pit, planner), rs in sorted(groups.items()):
        row = {'map': pit, 'planner': planner, 'runs': str(len(rs)), 'landed': str(sum(r['status'] == 'landed' for r in rs))}
        for k in NUMBERS:
            vals = [r[k] for r in rs if r.get(k) is not None]
            row[k] = '-' if not vals else fmt(statistics.mean(vals), k) + (' ± ' + fmt(statistics.stdev(vals), k) if len(vals) > 1 else '')
        row['min_clear_m'] = fmt(min(r['min_clear_m'] for r in rs if r.get('min_clear_m') is not None), 'min_clear_m') if any(r.get('min_clear_m') is not None for r in rs) else '-'
        summary.append(row)
    print('\nPer map and planner: mean ± standard deviation; min_clear_m is the closest over all runs')
    print(table(summary, ['map', 'planner', 'runs', 'landed'] + NUMBERS))
    if args.out:
        with open(os.path.expanduser(args.out), 'w', newline='') as f:
            w = csv.DictWriter(f, COLUMNS)
            w.writeheader()
            for r in runs: w.writerow({k: r.get(k, '') for k in COLUMNS})
        print('\nWritten: ' + args.out)


if __name__ == '__main__':
    main()
