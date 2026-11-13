#!/usr/bin/env python3
"""The results table and the notable moments of every run in a run_batch.py folder, as Markdown, with each moment's place
in the run's video, so you can go back and watch it.

  rosrun autonomous_flight run_highlights.py ~/mission_runs/batch_20261109_120000
  rosrun autonomous_flight run_highlights.py ~/mission_runs/batch_20261109_120000 --out ~/Downloads/trial_highlights.md
  rosrun autonomous_flight run_highlights.py ~/mission_runs/batch_x --exclude excavation_slab_epic_r2 excavation_slab_epic_r3

Moments come from three places:
  the mission events      [Mission] lines (DEP) and [epic_mission] lines (EPIC): stuck, return trouble, why it stopped
  the planners' warnings  trajectory failures, collision replans, EPIC's "not in corridor", grouped into bursts
  the mission CSV         failed legs, slow plans, long legs, depth and coverage milestones, close calls, hovering
Video times are positions in <run>_paths.mp4; mission times match the clock on the video's top-left label.
Moments marked ★ are the ones most worth watching.
"""
import argparse
import bisect
import csv
import glob
import json
import os
import re
import statistics
import sys

import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import summarize_runs as sr  # noqa: E402

MAPS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'cfg', 'dynamic_exploration', 'maps')
ANSI = re.compile(r'\x1b\[[0-9;]*m')
LINE = re.compile(r'^(\d+\.\d+) (.*)$')

# mission events: (pattern, label, worth watching). The first match wins; routine events are dropped
EVENTS = [
    (r'^(Ready|Taking off|Takeoff complete|Flying \d+ entry waypoints|Exploring|Entry waypoints done.*|Landing|Landed|Home reached|Saved this run.*)$', None, False),
    (r'Cannot find a way home|No way home|hovering', 'stuck', True),
    (r'Return leg aborted', 'return trouble', True),
    (r'retracing the flown path', 'EPIC gives up, retraces its path home', True),
    (r'EPIC sent no trajectory', 'EPIC stuck', True),
    (r'Landing: ground below not mapped', 'landing blind', True),
    (r'Entry waypoints aborted', 'entry aborted', True),
    (r'Plan dropped', 'plan dropped', False),
    (r'No reachable frontier, retrying', 'searching for a goal', False),
    (r'Exploration complete|EPIC finished', 'planner stops', False),
    (r'Returning home|Return home requested', 'return starts', False),
    (r'.', 'event', False),
]
# warnings, grouped into bursts: (pattern, label, worth watching)
WARNINGS = [
    (r'Rejected a non-finite trajectory|Non-finite trajectory target', 'bad trajectory rejected, hovering', True),
    (r'Unable to generate a feasible trajectory', 'no feasible trajectory', True),
    (r'Replan for collision', 'replanned: obstacle on the path', False),
    (r'goal of the current leg is occupied', 'goal turned out to be occupied', False),
    (r'current position not in corridor', 'EPIC: drone outside its flight corridor', True),
]
BURST_GAP = 20.0 # s between warnings of one burst
CLOSE_CALL = 0.35 # m to a mapped obstacle
CLOSE_CALL_WATCH = 0.25
HOVER_RADIUS = 0.3 # m
HOVER_TIME = 25.0 # s
SLOW_PLAN = 3.0 # s to choose a goal, and over twice the run's median
LONG_LEG = 20.0 # m flown for one DEP goal


def clock(seconds):
    if seconds is None: return '-'
    seconds = max(0, int(round(seconds)))
    return '%d:%02d' % (seconds // 60, seconds % 60)


def num(row, key):
    try:
        return float(row[key])
    except (KeyError, TypeError, ValueError):
        return None


def floor_z(pit):
    try:
        return float(yaml.safe_load(open(os.path.join(MAPS_DIR, pit + '.yaml')))['autonomous_flight']['floor_z'])
    except (OSError, KeyError, TypeError, ValueError):
        return -16.0 if pit == 'excavation1' else -10.0


class Timeline:
    """Converts between the mission clock, the wall clock and the video."""

    def __init__(self, run_dir):
        info = self.load_json(os.path.join(run_dir, 'run_info.json')) or {}
        self.start = info.get('mission_start_wall')
        self.pairs = []
        try:
            for r in csv.DictReader(open(os.path.join(run_dir, 'clock.csv'))):
                self.pairs.append((float(r['elapsed']), float(r['wall'])))
        except (OSError, ValueError, KeyError):
            pass
        self.pairs.sort()
        self.videos = [] # (first frame wall time, file)
        for meta in sorted(glob.glob(os.path.join(run_dir, '*_paths*.json'))):
            data = self.load_json(meta)
            mp4 = meta[:-5] + '.mp4'
            if data and os.path.exists(mp4): self.videos.append((data['video_start_epoch'], os.path.basename(mp4)))
        self.videos.sort()

    @staticmethod
    def load_json(path):
        try:
            return json.load(open(path))
        except (OSError, ValueError):
            return None

    @staticmethod
    def interpolate(pairs, x):
        if not pairs: return None
        if len(pairs) == 1: return pairs[0][1] + (x - pairs[0][0])
        xs = [p[0] for p in pairs]
        i = min(max(bisect.bisect_left(xs, x), 1), len(pairs) - 1)
        (x0, y0), (x1, y1) = pairs[i - 1], pairs[i]
        if x < x0 or x > x1 or x1 == x0: return (y0 if x < x0 else y1) + (x - (x0 if x < x0 else x1)) # past the ends: 1 to 1
        return y0 + (y1 - y0) * (x - x0) / (x1 - x0)

    def wall_of(self, mission_t):
        if self.pairs: return self.interpolate(self.pairs, mission_t)
        return None if self.start is None else self.start + mission_t

    def mission_of(self, wall):
        if self.pairs: return self.interpolate(sorted((w, e) for e, w in self.pairs), wall)
        return None if self.start is None else wall - self.start

    def video_of(self, wall):
        """(file, seconds into it) for a wall-clock time, or (None, None)."""
        if wall is None or not self.videos: return None, None
        started = [v for v in self.videos if v[0] <= wall + 1.0]
        start, name = started[-1] if started else self.videos[0]
        return name, wall - start


def log_lines(run_dir):
    for name in ('planner.log', 'sim.log'):
        try:
            for raw in open(os.path.join(run_dir, name), errors='replace'):
                m = LINE.match(raw.rstrip('\n'))
                if m: yield float(m.group(1)), ANSI.sub('', m.group(2))
        except OSError:
            pass


def log_moments(run_dir):
    """(wall, mission_t or None, label, text, watch) from the mission events and warning bursts."""
    out, bursts = [], {}
    for wall, text in log_lines(run_dir):
        m = re.search(r'\[Mission\]: (.*)$', text) or re.search(r'\[epic_mission\] (.*)$', text)
        if m:
            event = m.group(1).strip()
            for pattern, label, watch in EVENTS:
                if re.search(pattern, event):
                    if label: out.append((wall, None, label, event, watch))
                    break
            continue
        for pattern, label, watch in WARNINGS:
            if re.search(pattern, text):
                group = bursts.setdefault(label, [])
                if group and wall - group[-1][1] <= BURST_GAP:
                    group[-1][1], group[-1][2] = wall, group[-1][2] + 1
                else:
                    group.append([wall, wall, 1, watch])
                break
    for label, groups in bursts.items():
        for first, last, count, watch in groups:
            text = '%d times over %.0f s' % (count, last - first) if count > 1 else 'once'
            out.append((first, None, label, text, watch and count >= 3 or label.startswith('bad trajectory')))
    return out


def csv_moments(rows, pit, best):
    """(None, mission_t, label, text, watch) from the mission CSV."""
    out = []
    samples = [r for r in rows if r.get('type') == 'height' and num(r, 't_s') is not None]
    legs = [r for r in rows if r.get('type') == 'waypoint']
    epic = any(r.get('kind') == 'epic' for r in legs)
    floor = floor_z(pit)
    times = [num(r, 'plan_time_s') for r in legs if num(r, 'plan_time_s') is not None]
    slow = max(SLOW_PLAN, 2 * statistics.median(times)) if times else SLOW_PLAN

    # state changes the logs may not show (EPIC's states come only from here)
    for a, b in zip(samples, samples[1:]):
        if a['state'] != b['state'] and b['state'] in ('RETURNING', 'LANDING') and epic:
            out.append((None, num(b, 't_s'), 'state', a['state'].lower() + ' → ' + b['state'].lower(), False))

    for r in legs:
        t, end = num(r, 't_s'), (r.get('plan_end') or '').strip()
        goal = '(%s, %s, %s)' % (r.get('goal_x'), r.get('goal_y'), r.get('goal_z'))
        if end and end != 'reached':
            out.append((None, t, 'leg failed', 'goal %s %s: %s' % (r.get('n'), goal, end), 'not safe' in end or 'feasible' in end))
        plan = num(r, 'plan_time_s')
        if plan is not None and not epic and plan >= slow:
            out.append((None, t, 'slow plan', 'took %.1f s to choose goal %s' % (plan, r.get('n')), False))
        length = num(r, 'path_length')
        if length is not None and not epic and length >= LONG_LEG:
            out.append((None, t, 'long leg', '%.0f m to goal %s %s' % (length, r.get('n'), goal), True))

    flying = [r for r in samples if r['state'] in ('ENTRY', 'EXPLORING', 'INSPECTING', 'RETURNING')]
    if flying:
        for depth, label in ((-3.0, 'below the rim'), (floor / 2, 'halfway down'), (floor + 2.0, 'within 2 m of the floor')):
            hit = next((r for r in flying if num(r, 'z_m') is not None and num(r, 'z_m') <= depth), None)
            if hit: out.append((None, num(hit, 't_s'), 'depth', 'first %s (z %.1f m)' % (label, num(hit, 'z_m')), label.startswith('within')))
        low = min(flying, key=lambda r: num(r, 'z_m') if num(r, 'z_m') is not None else 1e9)
        out.append((None, num(low, 't_s'), 'deepest', 'lowest point: z %.1f m at (%.1f, %.1f), %.1f m above the floor'
                    % (num(low, 'z_m'), num(low, 'x_m'), num(low, 'y_m'), num(low, 'z_m') - floor), True))

    exploring = [r for r in samples if r['state'] == 'EXPLORING']
    if best and exploring:
        hit = next((r for r in exploring if (num(r, 'pit_block_pct') or 0) >= 0.9 * best), None)
        if hit: out.append((None, num(hit, 't_s'), 'coverage', 'reaches 90%% of the best coverage on this pit (%.0f%% of %.0f%%)'
                            % (num(hit, 'pit_block_pct'), best), False))
        series = sr.coverage_series(samples)
        run = {}
        sr.add_common_measures(run, series, best)
        if run.get('plateau_s') is not None:
            out.append((None, num(exploring[0], 't_s') + run['plateau_s'], 'plateau',
                        'coverage stops growing: under 1 point in the next 60 s (%.0f%%)' % run['plateau_pct'], False))

    # close calls, one per 15 s
    last = None
    for r in samples:
        c = num(r, 'min_clearance_m')
        if c is None or c >= CLOSE_CALL or r['state'] not in ('EXPLORING', 'RETURNING', 'ENTRY'): continue
        t = num(r, 't_s')
        if last and t - last[1] <= 15.0:
            if c < last[2]: last[2] = c
            last[1] = t
            continue
        last = [t, t, c, r]
        out.append(last)
    out = [m if isinstance(m, tuple) else (None, m[0], 'close call', '%.2f m from a mapped obstacle at z %.1f m' % (m[2], num(m[3], 'z_m')), m[2] < CLOSE_CALL_WATCH) for m in out]

    # hovering: barely moving for HOVER_TIME while it should be flying
    i = 0
    while i < len(samples):
        r = samples[i]
        if r['state'] not in ('EXPLORING', 'RETURNING') or num(r, 'x_m') is None:
            i += 1
            continue
        p = (num(r, 'x_m'), num(r, 'y_m'), num(r, 'z_m'))
        j = i
        while j + 1 < len(samples) and samples[j + 1]['state'] == r['state'] and num(samples[j + 1], 'x_m') is not None and \
                sum((a - b) ** 2 for a, b in zip(p, (num(samples[j + 1], 'x_m'), num(samples[j + 1], 'y_m'), num(samples[j + 1], 'z_m')))) ** 0.5 <= HOVER_RADIUS:
            j += 1
        held = num(samples[j], 't_s') - num(r, 't_s')
        if held >= HOVER_TIME:
            out.append((None, num(r, 't_s'), 'hovering', '%.0f s in place at (%.1f, %.1f, %.1f) while %s' % (held, p[0], p[1], p[2], r['state'].lower()), True))
            i = j + 1
        else:
            i += 1
    return out


def run_section(run_dir, row, best_by_map, summary):
    name = os.path.basename(run_dir)
    pit, preset = row.get('map', ''), row.get('preset', '')
    tl = Timeline(run_dir)
    files = sr_files(run_dir)
    rows = list(csv.DictReader(open(files[0]))) if files else []
    moments = []
    for wall, mt, label, text, watch in log_moments(run_dir):
        moments.append((wall, tl.mission_of(wall), label, text, watch))
    for _, mt, label, text, watch in csv_moments(rows, pit, best_by_map.get(pit)):
        moments.append((tl.wall_of(mt), mt, label, text, watch))
    moments.sort(key=lambda m: (m[1] if m[1] is not None else 1e9))

    s = summary or {}
    head = '### %s · %s · run %s' % (pit, preset, row.get('repeat', '?'))
    facts = [row.get('status', '?')]
    if s.get('mapped_pct') is not None: facts.append('%.0f%% of the pit mapped' % s['mapped_pct'])
    if s.get('t90_s') is not None: facts.append('90%% of best at %s of exploring' % clock(s['t90_s']))
    if s.get('mission_s') is not None: facts.append('mission %s' % clock(s['mission_s']))
    lines = [head, '', ', '.join(facts) + '.', '']
    if tl.videos:
        lines.append('Video: ' + ', '.join('[%s](%s/%s)' % (v, name, v) for _, v in tl.videos))
    else:
        lines.append('Video: none recorded (see %s/record.log)' % name)
    lines += ['', '| | Video | Mission | What | Detail |', '|---|---|---|---|---|']
    for wall, mt, label, text, watch in moments:
        video, at = tl.video_of(wall)
        where = clock(at) if video and at is not None and at >= 0 else '-'
        if video and len(tl.videos) > 1 and where != '-': where += ' (%s)' % video
        lines.append('| %s | %s | %s | %s | %s |' % ('★' if watch else '', where, clock(mt), label, text.replace('|', '/')))
    if not moments: lines.append('| | - | - | - | nothing logged |')
    return '\n'.join(lines), [(pit, preset, row.get('repeat'), m) for m in moments if m[4]]


def sr_files(run_dir):
    return sorted(glob.glob(os.path.join(run_dir, '*_run[0-9]*.csv')))


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('batch', help='a run_batch.py output folder')
    parser.add_argument('--out', help='Markdown file to write (default <batch>/highlights.md)')
    parser.add_argument('--exclude', nargs='+', default=[], metavar='RUN',
                        help='runs left out of the results table (their timelines stay), e.g. extra repeats')
    args = parser.parse_args()
    batch = os.path.abspath(os.path.expanduser(args.batch))
    out = os.path.abspath(os.path.expanduser(args.out)) if args.out else os.path.join(batch, 'highlights.md')
    results = list(csv.DictReader(open(os.path.join(batch, 'results.csv'))))
    order = {'excavation1': 0, 'excavation_long': 1, 'excavation_slab': 2}
    planner_order = {'baseline': 0, 'gain': 1, 'epic': 2}
    results.sort(key=lambda r: (order.get(r['map'], 9), planner_order.get(r['preset'], 9), int(r['repeat'] or 0)))

    # the numbers, as summarize_runs.py computes them
    summaries = {}
    for r in results:
        files = sr_files(os.path.join(batch, r['run']))
        if files: summaries[r['run']] = sr.summarize(files[0])
    best = {}
    for s in summaries.values():
        if s.get('series'): best[s['map']] = max(best.get(s['map'], 0.0), max(x[2] for x in s['series']))
    for s in summaries.values():
        sr.add_common_measures(s, s.get('series'), best.get(s['map'], 0.0))
        for share in (93, 95): # distance flown to higher coverage, where revisiting starts to cost
            hit = next((x for x in s.get('series') or [] if x[2] >= share / 100.0 * best.get(s['map'], 0.0)), None)
            s['path%d_m' % share] = hit[1] if hit else None

    doc = ['# Trial highlights: ' + os.path.basename(batch), '',
           'Every run of the batch, with the moments worth going back to. **Video** is the position in that run\'s '
           'no-voxel recording; **Mission** matches the clock on the video\'s top-left label. ★ marks the moments most worth '
           'watching. Made by `run_highlights.py` from the planners\' logs and mission CSVs.', '']

    # results table: mean ± sd per pit and planner
    cols = [('t90_s', 'Time to 90% of best (s)', 0), ('path90_m', 'Distance to 90% (m)', 0), ('path93_m', 'Distance to 93% (m)', 0),
            ('path95_m', 'Distance to 95% (m)', 0), ('path_m', 'Distance at own stop (m)', 0), ('at120_pct', 'Mapped at 120 s (%)', 0),
            ('plateau_s', 'Plateau (s)', 0), ('plateau_pct', 'Mapped at plateau (%)', 0), ('mapped_pct', 'Mapped at own stop (%)', 0),
            ('explore_s', 'Own stop (s)', 0), ('plan_s', 'Time per plan (s)', 2), ('min_clear_m', 'Closest obstacle (m)', 2)]
    doc += ['## Results', '', 'Best coverage per pit, the reference for "90% of best": ' +
            ', '.join('%s %.1f%%' % kv for kv in sorted(best.items(), key=lambda kv: order.get(kv[0], 9))) + '. '
            'Mean ± standard deviation over the runs that explored; (k/n) marks a cell only k of the n runs reached, so its mean '
            'is over those k. Distance to 93% and 95% is to that share of the best coverage. Closest obstacle is the minimum over all runs. '
            'EPIC times are real seconds (MARSIM), the others simulation seconds (Gazebo).', '',
            '| Pit | Planner | Runs (landed) | ' + ' | '.join(c[1] for c in cols) + ' |', '|' + '---|' * (3 + len(cols))]
    groups = {}
    for r in results:
        s = summaries.get(r['run'])
        if s and 'explore_s' in s and r['run'] not in args.exclude: groups.setdefault((r['map'], r['preset']), []).append((r, s))
    for (pit, preset), items in sorted(groups.items(), key=lambda kv: (order.get(kv[0][0], 9), planner_order.get(kv[0][1], 9))):
        cells = []
        for key, _, dec in cols:
            vals = [s[key] for _, s in items if s.get(key) is not None]
            if not vals: cells.append('-')
            elif key == 'min_clear_m': cells.append('%.*f' % (dec, min(vals)))
            elif len(vals) == 1: cells.append('%.*f' % (dec, vals[0]))
            else: cells.append('%.*f ± %.*f' % (dec, statistics.mean(vals), dec, statistics.stdev(vals)))
            if vals and key != 'min_clear_m' and len(vals) < len(items): cells[-1] += ' (%d/%d)' % (len(vals), len(items))
        landed = sum(r['status'] in ('landed', 'timeout_then_landed') for r, _ in items)
        doc.append('| %s | %s | %d (%d) | %s |' % (pit, preset, len(items), landed, ' | '.join(cells)))
    excluded = [r['run'] for r in results if r['run'] in args.exclude]
    if excluded:
        doc += ['', 'Left out of this table, still in the timelines below: ' + ', '.join(excluded) + '.']
    failed = [r for r in results if r['run'] not in summaries or 'explore_s' not in summaries[r['run']]]
    if failed:
        doc += ['', 'Runs left out (no exploring logged): ' + ', '.join('%s (%s)' % (r['run'], r['status']) for r in failed) + '.']
    doc.append('')

    sections, picks = [], []
    for r in results:
        text, watch = run_section(os.path.join(batch, r['run']), r, best, summaries.get(r['run']))
        sections.append(text)
        picks += [(r['run'], w) for w in watch]

    doc += ['## Worth watching', '', 'The ★ moments of every run, in one list.', '',
            '| Run | Video | Mission | What | Detail |', '|---|---|---|---|---|']
    for run, (pit, preset, repeat, m) in picks:
        tl = Timeline(os.path.join(batch, run))
        video, at = tl.video_of(m[0])
        link = '[%s](%s/%s)' % (clock(at), run, video) if video and at is not None and at >= 0 else '-'
        doc.append('| %s | %s | %s | %s | %s |' % (run, link, clock(m[1]), m[2], m[3].replace('|', '/')))
    doc += ['', '## Runs', '']
    doc += [s + '\n' for s in sections]
    with open(out, 'w') as f:
        f.write('\n'.join(doc))
    print('Written: ' + out)


if __name__ == '__main__':
    main()
