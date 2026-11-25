#!/usr/bin/env python3
"""Sanity check for per-column height options ("slices") as 3D graph nodes, on the true geometry of a world.

For each column of a 2 m node grid, the free vertical gaps where the drone's safety box (1 m) fits become height
options. Schemes compared:
  one      one option per gap, at its middle
  split3   gaps cut into pieces of at most 3 m, one option per piece
  split5   the same with 5 m pieces (about the tilted Mid-360's vertical view at 10 m)
  grid     every 2 m grid point in free configuration space (the full 3D graph, for reference)
Options in neighbouring columns (8-neighbours) are joined when the straight segment between them stays in free
configuration space, options in one gap of one column are joined to each other. Two kinds of planning gap:
  - nodes not connected to the start's component (unreachable on the graph although reachable in the world)
  - surfaces no connected node sees: from every connected node the tilted Mid-360 (23.5 deg, -7..52 deg FOV) is cast
    against the true boxes at 4 headings; a surface voxel counts as seen within 20 m (the map's raycast length) and
    as inspected within 10 m (the monitor's inspect range)

  slice_check.py --world excavation_slab --out /tmp/slices
"""
import argparse
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src'))
from voxel_world.course import Box, _dilate6, _flood, _rasterize  # noqa: E402
from voxel_world.fast import BoxScene, mid360_rays  # noqa: E402
from voxel_world.world import load_sdf_boxes  # noqa: E402

SRC = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
# node region: the pits' scoring region in x, y (maps/<map>.yaml progress_region), floor to 3 m above the rim
WORLDS = {
    'excavation_slab': dict(lo=[0.2, 5.2, -10.0], hi=[9.8, 34.8, 3.0], start=[5.0, 8.0, -9.4]),
    'excavation_long': dict(lo=[0.2, 5.2, -10.0], hi=[13.8, 34.8, 3.0], start=[10.0, 8.0, -9.4]),
}
MARGIN = np.array([0.6, 0.6, 0.4])  # grid reaches past the node region so pit walls and floor are in it
SCHEMES = [('one', 'one', None), ('split3', 'split', 3.0), ('split5', 'split', 5.0), ('grid', 'grid', 2.0)]


def load(world):
    path = os.path.join(SRC, 'uav_simulator/models/%s/%s.sdf' % (world, world))
    boxes, raw = [], load_sdf_boxes(path)
    for b in raw:
        if abs(b.rotation[2, 2] - 1) > 1e-6:
            raise ValueError('%s is tilted: this check handles yaw only' % b.name)
        boxes.append(Box(b.name, np.asarray(b.translation, float), np.asarray(b.size, float),
                         float(np.arctan2(b.rotation[1, 0], b.rotation[0, 0]))))
    return boxes, BoxScene(raw)


def intervals(col):
    d = np.diff(np.r_[0, col.astype(int), 0])
    return list(zip(np.flatnonzero(d == 1), np.flatnonzero(d == -1)))


def build(cfree, glo, res, node_xy, zlo, zhi, mode, spacing):
    """Options for every column: arrays P (n,3), column index, gap index."""
    P, col, gap = [], [], []
    for ci, (x, y) in enumerate(node_xy):
        ix, iy = int((x - glo[0]) / res), int((y - glo[1]) / res)
        for gi, (a, b) in enumerate(intervals(cfree[:, iy, ix])):
            z0, z1 = max(glo[2] + a * res, zlo), min(glo[2] + b * res, zhi)
            if z1 <= z0:
                continue
            if mode == 'grid':
                zs = [z for z in np.arange(zlo + spacing / 2, zhi, spacing) if z0 <= z < z1]
            else:
                n = 1 if mode == 'one' else max(1, int(math.ceil((z1 - z0) / spacing)))
                zs = [z0 + (z1 - z0) * (k + 0.5) / n for k in range(n)]
            for z in zs:
                P.append((x, y, z)); col.append(ci); gap.append(gi)
    return np.array(P), np.array(col), np.array(gap)


def segment_free(cfree, glo, res, p, q, step=0.1):
    n = max(2, int(np.linalg.norm(q - p) / step) + 1)
    idx = np.floor((p + (q - p) * np.linspace(0, 1, n)[:, None] - glo) / res).astype(int)
    if np.any(idx < 0) or np.any(idx >= np.array(cfree.shape[::-1])):
        return False
    return bool(cfree[idx[:, 2], idx[:, 1], idx[:, 0]].all())


def connect(cfree, glo, res, P, col, gap, cxy, sp, start):
    parent = list(range(len(P)))

    def find(u):
        while parent[u] != u:
            parent[u] = parent[parent[u]]
            u = parent[u]
        return u
    edges = 0
    for i in range(len(P)):
        for j in range(i + 1, len(P)):
            if col[i] == col[j]:
                if gap[i] != gap[j]:
                    continue
            elif np.abs(cxy[col[i]] - cxy[col[j]]).max() > sp * 1.01:
                continue
            if segment_free(cfree, glo, res, P[i], P[j]):
                parent[find(i)] = find(j)
                edges += 1
    s = int(np.argmin(np.linalg.norm(P - start, axis=1)))
    return np.array([find(i) == find(s) for i in range(len(P))]), edges


def visibility(scene, nodes, surface, glo, res, rays, pitch=23.5, offset=0.13):
    """Surface voxels hit by the tilted Mid-360 from any node at 4 headings: (within 20 m, within 10 m) masks."""
    seen20, seen10 = np.zeros_like(surface), np.zeros_like(surface)
    a = math.radians(pitch)
    tilt = np.array([[math.cos(a), 0, math.sin(a)], [0, 1, 0], [-math.sin(a), 0, math.cos(a)]])
    shape = np.array(surface.shape[::-1])
    for p in nodes:
        o = p + [0, 0, offset]
        for yaw in (0.0, 0.5 * math.pi, math.pi, 1.5 * math.pi):
            c, s = math.cos(yaw), math.sin(yaw)
            d = rays @ (np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]]) @ tilt).T
            r = scene.raycast(o, d, 20.0)
            ok = np.isfinite(r)
            hit = o + d[ok] * (r[ok, None] + 0.05)  # just inside the surface
            idx = np.floor((hit - glo) / res).astype(int)
            inb = np.all((idx >= 0) & (idx < shape), axis=1)
            idx, rr = idx[inb], r[ok][inb]
            seen20[idx[:, 2], idx[:, 1], idx[:, 0]] = True
            near = rr <= 10.0
            seen10[idx[near, 2], idx[near, 1], idx[near, 0]] = True
    return seen20 & surface, seen10 & surface


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--world', choices=sorted(WORLDS), required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--node-spacing', type=float, default=2.0)
    ap.add_argument('--robot-size', type=float, default=1.0)
    ap.add_argument('--res', type=float, default=0.2)
    ap.add_argument('--rays', type=int, default=4000)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    w = WORLDS[a.world]
    nlo, nhi, res = np.array(w['lo']), np.array(w['hi']), a.res
    glo, ghi = nlo - MARGIN, nhi + MARGIN * [1, 1, 0]
    boxes, scene = load(a.world)
    occ = _rasterize(boxes, glo, ghi - glo, res, inflate=0.0)
    cfree = ~_rasterize(boxes, glo, ghi - glo, res, inflate=a.robot_size / 2)
    cfree[int(np.floor((nhi[2] - glo[2] - a.robot_size / 2) / res)):] = False  # stay below the region's top
    start = np.array(w['start'])
    reach = _flood(cfree, tuple(np.floor((start - glo) / res).astype(int)[::-1]))
    # surfaces to see: obstacle voxels next to free space, inside the pit (below the rim) and inside the node region in x, y
    zc = glo[2] + (np.arange(occ.shape[0]) + 0.5) * res
    yc = glo[1] + (np.arange(occ.shape[1]) + 0.5) * res
    xc = glo[0] + (np.arange(occ.shape[2]) + 0.5) * res
    inside = ((zc < 0.0)[:, None, None] & ((yc > nlo[1] - 0.6) & (yc < nhi[1] + 0.6))[None, :, None]
              & ((xc > nlo[0] - 0.6) & (xc < nhi[0] + 0.6))[None, None, :])
    surface = occ & _dilate6(~occ) & inside
    print('%s: reachable C-space %.0f m3, pit surface voxels %d' % (a.world, reach.sum() * res ** 3, surface.sum()))

    sp = a.node_spacing
    xs, ys = np.arange(nlo[0] + sp / 2, nhi[0], sp), np.arange(nlo[1] + sp / 2, nhi[1], sp)
    node_xy = [(x, y) for x in xs for y in ys]
    cxy = np.array(node_xy)
    rays = mid360_rays(count=a.rays)
    summary, results = dict(world=a.world, columns=len(node_xy), surface_voxels=int(surface.sum())), {}
    for name, mode, spacing in SCHEMES:
        P, col, gap = build(cfree, glo, res, node_xy, nlo[2], nhi[2], mode, spacing)
        main_c, edges = connect(cfree, glo, res, P, col, gap, cxy, sp, start)
        s20, s10 = visibility(scene, P[main_c], surface, glo, res, rays)
        results[name] = dict(P=P, main=main_c, unseen=surface & ~s20, uninspected=surface & ~s10)
        summary[name] = dict(nodes=len(P), per_column=round(len(P) / len(node_xy), 2), disconnected=int((~main_c).sum()),
                             edges=edges, surface_seen_pct=round(100 * s20.sum() / surface.sum(), 2),
                             surface_within_10m_pct=round(100 * s10.sum() / surface.sum(), 2))
        print('%-7s %s' % (name, json.dumps(summary[name])), flush=True)
    json.dump(summary, open(os.path.join(a.out, '%s_summary.json' % a.world), 'w'), indent=1)
    plot(a, glo, ghi, nlo, nhi, res, occ, reach, results, summary, xs, ys)


def plot(a, glo, ghi, nlo, nhi, res, occ, reach, results, summary, xs, ys):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.colors import ListedColormap

    cuts = [('x', xs[len(xs) // 2])] + [('y', ys[k]) for k in (1, len(ys) // 3, len(ys) // 2, 2 * len(ys) // 3, len(ys) - 2)]
    show = ['one', 'split5', 'split3']
    fig, axes = plt.subplots(len(show), len(cuts), figsize=(22, 4.1 * len(show)),
                             gridspec_kw=dict(width_ratios=[3] + [1] * (len(cuts) - 1)))
    cmap = ListedColormap(['white', '#d6e9f8', '#8d939c', '#e74c3c'])
    for r_i, name in enumerate(show):
        r = results[name]
        for c_i, (axis, at) in enumerate(cuts):
            ax = axes[r_i, c_i]
            if axis == 'x':
                i = int((at - glo[0]) / res)
                sl = lambda g: g[:, :, i]  # noqa: E731
                ext, h, m = [glo[1], ghi[1], glo[2], ghi[2]], 1, np.abs(r['P'][:, 0] - at) < 0.01
            else:
                j = int((at - glo[1]) / res)
                sl = lambda g: g[:, j, :]  # noqa: E731
                ext, h, m = [glo[0], ghi[0], glo[2], ghi[2]], 0, np.abs(r['P'][:, 1] - at) < 0.01
            img = np.zeros(sl(occ).shape, int)
            img[sl(reach)] = 1
            img[sl(occ)] = 2
            # unseen surface, thickened by one voxel so it shows at this scale
            un = sl(r['unseen'])
            img[un | np.roll(un, 1, 0) & sl(occ) | np.roll(un, 1, 1) & sl(occ)] = 3
            ax.imshow(img, origin='lower', extent=ext, cmap=cmap, vmin=0, vmax=3, interpolation='nearest', aspect='auto')
            P = r['P']
            ax.scatter(P[m & r['main'], h], P[m & r['main'], 2], s=14, c='#1e8449', zorder=3)
            ax.scatter(P[m & ~r['main'], h], P[m & ~r['main'], 2], s=30, c='#c0392b', marker='x', zorder=3)
            ax.axhline(0.0, color='#555555', lw=0.6, ls=':')
            ax.tick_params(labelsize=7)
            if r_i == 0:
                ax.set_title(('side cut along the pit at x = %.1f m' if axis == 'x' else 'cross-cut at y = %.1f m') % at, fontsize=9)
            if c_i == 0:
                s = summary[name]
                ax.set_ylabel('%s\n%d nodes (%.1f/column)\n%d not connected\nsurface seen %.1f%%, within 10 m %.1f%%'
                              % (name, s['nodes'], s['per_column'], s['disconnected'], s['surface_seen_pct'],
                                 s['surface_within_10m_pct']), fontsize=8.5)
    g = summary['grid']
    fig.suptitle('%s: height options per 2 m column on the true geometry. light blue: reachable space, grey: obstacles, '
                 'red: pit surface no connected node sees, green dot: node, red x: node not connected.  '
                 'Full 2 m grid for reference: %d nodes, surface seen %.1f%%, within 10 m %.1f%%'
                 % (a.world, g['nodes'], g['surface_seen_pct'], g['surface_within_10m_pct']), fontsize=10)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.savefig(os.path.join(a.out, '%s_slices.png' % a.world), dpi=80)


if __name__ == '__main__':
    main()
