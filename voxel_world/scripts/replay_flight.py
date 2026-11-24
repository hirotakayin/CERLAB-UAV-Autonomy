#!/usr/bin/env python3
"""Replay a recorded Gazebo flight through the Python map and compare coverage with what epic_monitor_node logged.

Needs a bag with /CERLAB/quadcopter/odom, /cloud_registered_body and the mission state topic, and the run's CSV
from the monitor. Three replays, each scored with the monitor's own numbers (pit, block, floor, surface %):

  gazebo    the recorded Gazebo scans into the Python map: checks the port of map_manager and of the scoring
  boxes     the same scans re-cast against the SDF boxes (BoxScene) with the same Livox rays: checks the world and
            sensor model
  training  the boxes with training shortcuts: coarser map (--train-res), an even ray grid (--train-rays), and only
            the scans --train-every apart

The pass test for stage 1 is that boxes and training follow the Gazebo curves within a few percent.

Example:
  replay_flight.py --bag run.bag --csv 2026..._run1.csv --out replay/   (defaults: excavation_long, 23.5 deg)
"""
import argparse
import csv
import glob
import json
import math
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src'))
from voxel_world.fast import BoxScene, OccupancyMap, PitScore, mid360_rays  # noqa: E402
from voxel_world.world import load_mid360_directions, rotation_from_quaternion  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.normpath(os.path.join(HERE, '..', '..'))
# map bounds and scoring region of each pit: the scoring map's yaml (cfg/dynamic_exploration/maps/<map>*.yaml)
PITS = {
    'excavation_long': dict(origin=[-23, -45, -10.5], size=[46, 90, 15],
                            region=([0.2, 5.2, -9.8], [13.8, 34.8, 0.0]), floor_z=-10.0),
    'excavation_slab': dict(origin=[-23, -45, -10.5], size=[46, 90, 15],
                            region=([0.2, 5.2, -9.8], [9.8, 34.8, 0.0]), floor_z=-10.0),
}
METRICS = ['pit_block_pct', 'pit_pct', 'floor_pct', 'surface_pct']


def read_cloud(msg):
    """PointCloud2 to (N,3) float64, in point order (no-returns stay as rows of zeros)."""
    off = {f.name: f.offset for f in msg.fields}
    dt = np.dtype({'names': ['x', 'y', 'z'], 'formats': ['<f4'] * 3,
                   'offsets': [off['x'], off['y'], off['z']], 'itemsize': msg.point_step})
    a = np.frombuffer(msg.data, dtype=dt, count=msg.width * msg.height)
    return np.stack((a['x'], a['y'], a['z']), axis=1).astype(np.float64)


def read_bag(path, state_topic):
    import rosbag
    odom_t, odom_p, odom_q, scans, start = [], [], [], [], None
    with rosbag.Bag(path) as bag:
        for topic, msg, t in bag.read_messages(topics=['/CERLAB/quadcopter/odom', '/cloud_registered_body',
                                                       state_topic]):
            if topic == state_topic:
                if start is None and msg.data != 'IDLE':
                    start = t.to_sec()  # the monitor starts its clock when the state leaves IDLE
            elif topic == '/cloud_registered_body':
                scans.append((msg.header.stamp.to_sec(), read_cloud(msg)))
            else:
                p, q = msg.pose.pose.position, msg.pose.pose.orientation
                odom_t.append(msg.header.stamp.to_sec())
                odom_p.append((p.x, p.y, p.z))
                odom_q.append((q.x, q.y, q.z, q.w))
    return np.array(odom_t), np.array(odom_p), np.array(odom_q), scans, start


def sensor_pose(odom, stamp, pitch_deg, z_offset=0.13):
    """Lidar origin and world-from-lidar rotation at the odometry message nearest to stamp."""
    t, p, q = odom
    i = int(np.clip(np.searchsorted(t, stamp), 1, len(t) - 1))
    i = i - 1 if stamp - t[i - 1] < t[i] - stamp else i
    body = rotation_from_quaternion(*q[i])
    a = math.radians(pitch_deg)
    pitch = np.array([[math.cos(a), 0, math.sin(a)], [0, 1, 0], [-math.sin(a), 0, math.cos(a)]])
    return p[i] + body @ np.array([0, 0, z_offset]), body @ pitch


class PatternMatcher:
    """Which part of the Livox pattern a recorded scan used: the plugin steps through it 20,000 rows per scan
    (every 2nd row published), so each scan is one of 40 frames. Picked by matching the returned points' directions."""

    def __init__(self, csv_path, rows_per_scan=20000, downsample=2):
        self.full = load_mid360_directions(csv_path, downsample=1, samples=None)
        self.rows, self.down = rows_per_scan, downsample
        self.frames = len(self.full) // rows_per_scan
        self.cache = [self.full[k * rows_per_scan:(k + 1) * rows_per_scan:downsample] for k in range(self.frames)]

    def directions(self, points):
        r = np.linalg.norm(points, axis=1)
        ret = np.flatnonzero(r > 0.1)
        sample = ret[:: max(1, len(ret) // 300)]
        unit = points[sample] / r[sample, None]
        score = [np.mean(np.sum(f[sample] * unit, axis=1)) for f in self.cache]
        k = int(np.argmax(score))
        return self.cache[k], score[k]


def replay(scans, odom, start, sample_t, mode, pitch, pit, sdf, pattern, train_res, train_rays, train_every):
    res = train_res if mode == 'training' else 0.2
    omap = OccupancyMap(pit['origin'], pit['size'], res)
    score = PitScore(omap, *pit['region'], floor_z=pit['floor_z'])
    scene = BoxScene.from_sdf(sdf) if mode != 'gazebo' else None
    grid = mid360_rays(count=train_rays) if mode == 'training' else None
    rows, k, last_pos, worst_match, t_used = [], 0, None, 1.0, 0.0
    for t in sample_t:
        while k < len(scans) and scans[k][0] <= start + t:
            stamp, pts = scans[k]
            k += 1
            origin, rot = sensor_pose(odom, stamp, pitch)
            if mode == 'training' and last_pos is not None and np.linalg.norm(origin - last_pos) < train_every:
                continue
            last_pos = origin
            tic = time.time()
            if mode == 'gazebo':
                world = pts @ rot.T + origin
                world[np.linalg.norm(pts, axis=1) < 1e-6] = np.nan  # no return
            else:
                if mode == 'boxes':
                    dirs, match = pattern.directions(pts)
                    worst_match = min(worst_match, match)
                else:
                    dirs = grid
                d = dirs @ rot.T
                r = scene.raycast(origin, d, 40.0)  # the plugin's max range; occMap caps rays at 20 m itself
                world = origin + d * np.where(np.isfinite(r), r, np.nan)[:, None]
            omap.integrate(origin, world)
            t_used += time.time() - tic
        rows.append(dict(t=t, **score()))
    return rows, dict(scans=omap.scans, ms_per_scan=1e3 * t_used / max(1, omap.scans), worst_pattern_match=worst_match)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--bag', required=True)
    ap.add_argument('--csv', required=True, help="the monitor's run CSV")
    ap.add_argument('--map', default='excavation_long', choices=sorted(PITS))
    ap.add_argument('--pitch', type=float, default=23.5)
    ap.add_argument('--state-topic', default='/ariadne_mission/state')
    ap.add_argument('--modes', default='gazebo,boxes,training')
    ap.add_argument('--train-res', type=float, default=0.4)
    ap.add_argument('--train-rays', type=int, default=1500)
    ap.add_argument('--train-every', type=float, default=0.0, help='m between training scans (0 = every scan)')
    ap.add_argument('--pattern', default=os.path.join(SRC, 'uav_simulator/src/livox_lidar/scan_mode/mid360-real-centr.csv'))
    ap.add_argument('--out', default='.')
    a = ap.parse_args()

    pit = PITS[a.map]
    sdf = os.path.join(SRC, 'uav_simulator/models/%s/%s.sdf' % (a.map, a.map))
    ref = [r for r in csv.DictReader(open(a.csv)) if r['type'] == 'height']
    sample_t = [float(r['t_s']) for r in ref]
    print('reading %s' % a.bag)
    odom_t, odom_p, odom_q, scans, start = read_bag(a.bag, a.state_topic)
    if start is None:
        sys.exit('no mission start in the bag (%s never left IDLE)' % a.state_topic)
    odom = (odom_t, odom_p, odom_q)
    print('%d scans, %d odometry messages, mission start at %.2f s' % (len(scans), len(odom_t), start))
    pattern = PatternMatcher(a.pattern)

    os.makedirs(a.out, exist_ok=True)
    curves, info = {'monitor': [dict(t=float(r['t_s']), **{m: float(r[m]) if r[m] else float('nan') for m in METRICS})
                                for r in ref]}, {}
    for mode in a.modes.split(','):
        tic = time.time()
        curves[mode], info[mode] = replay(scans, odom, start, sample_t, mode, a.pitch, pit, sdf, pattern,
                                          a.train_res, a.train_rays, a.train_every)
        info[mode]['wall_s'] = round(time.time() - tic, 1)
        print('%-8s %s' % (mode, info[mode]))

    with open(os.path.join(a.out, 'replay_curves.csv'), 'w') as f:
        w = csv.writer(f)
        w.writerow(['source', 't_s'] + METRICS)
        for src, rows in curves.items():
            for r in rows:
                w.writerow([src, '%.2f' % r['t']] + ['%.2f' % r[m] for m in METRICS])

    # agreement with the monitor: final value and the largest gap over the flight, per metric
    summary = {}
    print('\n%-8s %-14s %8s %8s %8s' % ('source', 'metric', 'final', 'monitor', 'max gap'))
    for src in [s for s in curves if s != 'monitor']:
        summary[src] = dict(info[src])
        for m in METRICS:
            mine = np.array([r[m] for r in curves[src]])
            mon = np.array([r[m] for r in curves['monitor']])
            ok = np.isfinite(mine) & np.isfinite(mon)
            gap = float(np.max(np.abs(mine[ok] - mon[ok]))) if ok.any() else float('nan')
            summary[src][m] = dict(final=round(float(mine[-1]), 2), monitor=round(float(mon[-1]), 2), max_gap=round(gap, 2))
            print('%-8s %-14s %8.2f %8.2f %8.2f' % (src, m, mine[-1], mon[-1], gap))
    json.dump(summary, open(os.path.join(a.out, 'replay_summary.json'), 'w'), indent=1)

    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, axes = plt.subplots(1, len(METRICS), figsize=(4.2 * len(METRICS), 3.6), sharex=True)
        style = {'monitor': dict(color='#222222', lw=2.4), 'gazebo': dict(color='#1f77b4', lw=1.4),
                 'boxes': dict(color='#d62728', lw=1.4, ls='--'), 'training': dict(color='#2ca02c', lw=1.4, ls=':')}
        for ax, m in zip(axes, METRICS):
            for src, rows in curves.items():
                ax.plot([r['t'] for r in rows], [r[m] for r in rows], label=src, **style.get(src, {}))
            ax.set_title(m.replace('_pct', ' %').replace('_', ' '))
            ax.set_xlabel('time since start (s)')
            ax.grid(alpha=0.3)
        axes[0].legend(loc='lower right', fontsize=8)
        fig.suptitle('%s replay, lidar %.1f deg: Gazebo monitor vs Python map' % (a.map, a.pitch))
        fig.tight_layout()
        fig.savefig(os.path.join(a.out, 'replay_curves.png'), dpi=110)
        print('\nplot: %s' % os.path.join(a.out, 'replay_curves.png'))
    except ImportError:
        print('matplotlib not found: no plot')


if __name__ == '__main__':
    main()
