#!/usr/bin/env python3
"""How much of a course one or two viewpoints see: the user's worry that open rooms are seen whole from 1-2 spots.

For each course: line of sight from the start and from candidate viewpoints (configuration-space-free points on a
2.5 m grid at 1.5 m height steps) to a random 12k of the course's surface voxels, kept to the tilted Mid-360's
elevation band and 20 m range. Reports the share of the
course's surface voxels seen within 20 m from the start alone, and from the best 1 and best 2 viewpoints (greedy),
and the number of greedy viewpoints needed for 90% of surfaces.

  view_check.py ~/mission_runs/courses/test_v2 ~/mission_runs/courses/maze_v1 --out ~/mission_runs/courses/view_check.json
"""
import argparse
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src'))
from voxel_world.course import Box, _rasterize  # noqa: E402
from voxel_world.fast import BoxScene, mid360_rays  # noqa: E402
from voxel_world.world import BoxCollision  # noqa: E402


def load(folder):
    z = np.load(os.path.join(folder, 'course.npz'))
    boxes = [Box(str(k), c, s, float(y)) for k, c, s, y in zip(z['kind'], z['center'], z['size'], z['yaw'])]
    return z, boxes


def seen_by(scene, p, pts, lo_deg=-30.5, hi_deg=75.5, max_range=20.0):
    """Line of sight from p to each surface point: inside the tilted Mid-360's elevation band (23.5 deg tilt of a
    -7..52 deg FOV; the 4 headings cover every azimuth), within max_range, and no box in between."""
    v = pts - p
    dist = np.linalg.norm(v, axis=1)
    elev = np.degrees(np.arcsin(np.clip(v[:, 2] / np.maximum(dist, 1e-9), -1, 1)))
    cand = (dist <= max_range) & (elev >= lo_deg) & (elev <= hi_deg)
    out = np.zeros(len(pts), bool)
    k = np.nonzero(cand)[0]
    if len(k):
        r = scene.raycast(p, v[k] / dist[k, None], max_range)
        out[k] = ~np.isfinite(r) | (r >= dist[k] - 0.2)   # the first hit is the point's own voxel (or beyond)
    return out


def check(folder, rays=None, robot=1.0, step=2.5, sample=12000):
    z, boxes = load(folder)
    scene = BoxScene([BoxCollision('b%d' % i, b.size, np.array([[math.cos(b.yaw), -math.sin(b.yaw), 0], [math.sin(b.yaw), math.cos(b.yaw), 0], [0, 0, 1]]), b.center) for i, b in enumerate(boxes)])
    res, glo, room = float(z['resolution']), z['origin'], z['room']
    surface = z['surface']
    shape = np.array(surface.shape[::-1])
    zz, yy, xx = np.nonzero(surface)
    pick = np.random.default_rng(0).permutation(len(xx))[:sample]
    pts = glo + (np.stack([xx, yy, zz], 1)[pick] + 0.5) * res     # surface voxel centres, a random subset
    # nudge each point a little toward the free side so the ray to it is not grazing its own face
    expl = z['explorable']
    nudge = np.zeros_like(pts)
    for ax in range(3):
        for sgn in (1, -1):
            j = np.floor((pts - glo) / res).astype(int)
            j[:, ax] += sgn
            j = np.clip(j, 0, shape - 1)
            nudge[:, ax] += sgn * expl[j[:, 2], j[:, 1], j[:, 0]]
    pts = pts + 0.09 * nudge
    # candidates: C-free on a 2 m x 2 m x 1 m lattice, inside the explorable space
    cfree = ~_rasterize(boxes, np.zeros(3), room, 0.4, inflate=robot / 2)
    cands = []
    for x in np.arange(1.0, room[0], step):
        for y in np.arange(1.0, room[1], step):
            for h in np.arange(0.6, room[2] - 0.4, 1.5):
                i = np.floor(np.array([x, y, h]) / 0.4).astype(int)
                j = np.floor((np.array([x, y, h]) - glo) / res).astype(int)
                if cfree[i[2], i[1], i[0]] and expl[j[2], j[1], j[0]]:
                    cands.append([x, y, h])
    cands = np.array(cands)
    vis = np.array([seen_by(scene, p, pts) for p in cands])
    start_seen = seen_by(scene, z['start'], pts)
    n = len(pts)
    got, picks, curve = np.zeros(n, bool), [], []
    for _ in range(min(60, len(cands))):
        gain = (vis & ~got).sum(1)
        k = int(gain.argmax())
        if gain[k] == 0:
            break
        got |= vis[k]
        picks.append(k)
        curve.append(got.sum() / n)
        if curve[-1] >= 0.95 and len(curve) >= 2:
            break
    need = lambda q: next((i + 1 for i, v in enumerate(curve) if v >= q), None)
    return dict(course=os.path.basename(folder), candidates=len(cands), surface_cells=n,
                start=round(float(start_seen.mean()), 4), best1=round(curve[0], 4), best2=round(curve[1], 4) if len(curve) > 1 else None,
                views_for_90=need(0.90), views_for_95=need(0.95), any_view=round(float(vis.any(0).mean()), 4))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('sets', nargs='+')
    ap.add_argument('--start', default='floor', help='only course folders ending in _<start>')
    ap.add_argument('--out')
    a = ap.parse_args()
    rays = None
    out = {}
    for s in a.sets:
        s = os.path.expanduser(s)
        rows = []
        for f in sorted(os.listdir(s)):
            if os.path.isdir(os.path.join(s, f)) and f.endswith('_' + a.start):
                r = check(os.path.join(s, f), rays)
                rows.append(r)
                print('%-22s start %5.1f%%  best1 %5.1f%%  best2 %5.1f%%  views for 90%%: %s  95%%: %s' % (
                    r['course'], 100 * r['start'], 100 * r['best1'], 100 * (r['best2'] or 0), r['views_for_90'], r['views_for_95']), flush=True)
        med = lambda k: float(np.median([r[k] for r in rows if r[k] is not None]))
        print('== %s: median start %.1f%%, best1 %.1f%%, best2 %.1f%%, views for 90%% %.1f, 95%% %.1f' % (
            os.path.basename(s), 100 * med('start'), 100 * med('best1'), 100 * med('best2'), med('views_for_90'), med('views_for_95')))
        out[os.path.basename(s)] = rows
    if a.out:
        with open(os.path.expanduser(a.out), 'w') as f:
            json.dump(out, f, indent=1)


if __name__ == '__main__':
    main()
