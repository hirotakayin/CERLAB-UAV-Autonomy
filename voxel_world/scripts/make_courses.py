#!/usr/bin/env python3
"""Generate obstacle courses (voxel_world.course) into a folder, one subfolder per seed and start, with previews.

  make_courses.py --split test --count 20 --out ~/courses            # seeds 0-19, both start heights
  make_courses.py --seeds 1000 1001 --start ceiling --difficulty 0.8 --out /tmp/c
  make_courses.py --layout maze --split test --count 20 --out ~/mazes      # winding corridors and rooms

Each course folder holds course.npz, stats.json, model.sdf, model.config and world.sdf (see course.py), plus
preview.png. index.json in --out lists every course with its stats.
"""
import argparse
import json
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src'))
from voxel_world.course import SPLITS, CourseParams, generate, split_of  # noqa: E402

COLORS = {'room': '#b9bec7', 'pillar': '#8c6d4f', 'block': '#4a72b0', 'beam': '#b5523b', 'ledge': '#5a9a66',
          'floating': '#8a5aa6', 'wall': '#9c9a7c', 'maze': '#6b6f78', 'step': '#c9b48f', 'soffit': '#9fb8d6',
          'hurdle': '#d08a2e', 'lintel': '#3f7fd0', 'bar': '#c0392b', 'shelf': '#3fa66a'}


def footprint(b, axes):
    """Corners of a box's outline projected onto two world axes (0 x, 1 y, 2 z), yaw included."""
    c, s = np.cos(b.yaw), np.sin(b.yaw)
    hx, hy, hz = np.asarray(b.size) / 2
    corners = np.array([[x, y, z] for x in (-hx, hx) for y in (-hy, hy) for z in (-hz, hz)])
    rot = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    pts = corners @ rot.T + b.center
    p = pts[:, axes]
    if axes == [0, 1]:  # top view: the rotated rectangle itself
        q = np.array([[-hx, -hy], [hx, -hy], [hx, hy], [-hx, hy]])
        return q @ rot[:2, :2].T + b.center[:2]
    lo, hi = p.min(0), p.max(0)  # side views: the box's extent
    return np.array([[lo[0], lo[1]], [hi[0], lo[1]], [hi[0], hi[1]], [lo[0], hi[1]]])


def preview(course, path):
    if course.params.layout == 'maze':
        return preview_maze(course, path)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection  # noqa: F401 (registers the 3d projection)

    L, W, H = course.room
    fig = plt.figure(figsize=(16, 4.6))
    views = [([0, 1], 'top (x, y)', (L, W)), ([0, 2], 'side (x, z)', (L, H)), ([1, 2], 'side (y, z)', (W, H))]
    widths = [L, L, W, 0.9 * max(L, W)]
    gs = fig.add_gridspec(1, 4, width_ratios=widths)
    obstacles = [b for b in course.boxes if b.kind != 'room']
    for k, (axes, title, (a, b_)) in enumerate(views):
        ax = fig.add_subplot(gs[k])
        ax.add_patch(Polygon([[0, 0], [a, 0], [a, b_], [0, b_]], closed=True, fill=False, ec='#666666', lw=1.2))
        order = sorted(obstacles, key=lambda b: b.center[3 - sum(axes)] if axes != [0, 1] else b.center[2])
        for b in order:
            ax.add_patch(Polygon(footprint(b, axes), closed=True, fc=COLORS[b.kind], ec='#333333', lw=0.4, alpha=0.55))
        ax.plot(course.start[axes[0]], course.start[axes[1]], marker='*', ms=14, color='#e0218a', mec='white', mew=0.8)
        ax.set_xlim(-0.5, a + 0.5)
        ax.set_ylim(-0.5, b_ + 0.5)
        ax.set_aspect('equal')
        ax.set_title(title, fontsize=10)
        ax.tick_params(labelsize=8)
    ax = fig.add_subplot(gs[3], projection='3d')
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    for b in obstacles:
        c, s = np.cos(b.yaw), np.sin(b.yaw)
        rot = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        h = np.asarray(b.size) / 2
        v = np.array([[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)]) * h @ rot.T + b.center
        faces = [[v[i] for i in f] for f in ([0, 1, 3, 2], [4, 5, 7, 6], [0, 1, 5, 4], [2, 3, 7, 6], [0, 2, 6, 4], [1, 3, 7, 5])]
        ax.add_collection3d(Poly3DCollection(faces, facecolors=COLORS[b.kind], edgecolors='#333333', linewidths=0.2, alpha=0.6))
    ax.scatter(*course.start, marker='*', s=160, color='#e0218a')
    ax.set_xlim(0, L)
    ax.set_ylim(0, W)
    ax.set_zlim(0, H)
    ax.view_init(elev=28, azim=-55)
    ax.set_title('3D', fontsize=10)
    ax.tick_params(labelsize=7)
    st = course.stats()
    counts = ', '.join('%d %s' % (v, k + ('s' if v != 1 else '')) for k, v in st['obstacles'].items())
    fig.suptitle('course %d (%s split), %.1f x %.1f x %.1f m, start on the %s: %s; explorable %.0f m3'
                 % (course.seed, split_of(course.seed), L, W, H, course.params.start, counts, st['explorable_m3']), fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.93))
    fig.savefig(path, dpi=90)
    plt.close(fig)


def preview_maze(course, path):
    """Top view of the walls and obstacles, maps of the local ceiling and floor, and a 3D view."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    L, W, H = course.room
    fig = plt.figure(figsize=(17, 4.8))
    gs = fig.add_gridspec(1, 4, width_ratios=[L, L * 1.18, L * 1.18, 1.0 * max(L, W)])
    obstacles = [b for b in course.boxes if b.kind not in ('room', 'maze', 'step', 'soffit')]
    base = course.info.get('corridor_floor_m', 0.0)
    ax = fig.add_subplot(gs[0])
    for b in course.boxes:
        if b.kind in ('maze',):
            ax.add_patch(Polygon(footprint(b, [0, 1]), closed=True, fc=COLORS['maze'], ec='none'))
    for b in sorted(obstacles, key=lambda b: b.center[2]):
        ax.add_patch(Polygon(footprint(b, [0, 1]), closed=True, fc=COLORS[b.kind], ec='#222222', lw=0.4, alpha=0.8))
    from matplotlib.patches import Patch
    ax.legend(handles=[Patch(color=COLORS[k], label=k) for k in course.params.maze_counts], fontsize=7, loc='upper left',
              bbox_to_anchor=(0, -0.06), ncol=4, frameon=False)
    ax.plot(course.start[0], course.start[1], marker='*', ms=15, color='#e0218a', mec='white', mew=0.8)
    ax.set_title('top: walls (grey) and obstacles', fontsize=10)
    g = float(course.info['map_resolution'])
    for k, (name, data, cmap, lim) in enumerate([
            ('ceiling, m above the corridor floor', course.info['ceiling_map'] - base, 'viridis', (0, H - base)),
            ('floor, m above the corridor floor (sunken < 0)', course.info['floor_map'] - base, 'RdBu_r', (-6, 6))]):
        a2 = fig.add_subplot(gs[1 + k])
        m = np.ma.masked_invalid(data)
        im = a2.imshow(m, origin='lower', extent=(0, data.shape[1] * g, 0, data.shape[0] * g), cmap=cmap,
                       vmin=lim[0], vmax=lim[1], interpolation='nearest')
        a2.set_facecolor(COLORS['maze'])
        from mpl_toolkits.axes_grid1 import make_axes_locatable
        cax = make_axes_locatable(a2).append_axes('right', size='4%', pad=0.06)
        fig.colorbar(im, cax=cax).ax.tick_params(labelsize=7)
        a2.plot(course.start[0], course.start[1], marker='*', ms=15, color='#e0218a', mec='white', mew=0.8)
        a2.set_title(name, fontsize=10)
    for a2 in [a for a in fig.axes if a.get_title()][:3]:
        a2.set_xlim(-0.5, L + 0.5)
        a2.set_ylim(-0.5, W + 0.5)
        a2.set_aspect('equal')
        a2.tick_params(labelsize=8)
    ax = fig.add_subplot(gs[3], projection='3d')
    for b in course.boxes:
        if b.kind == 'room':
            continue
        h = np.asarray(b.size) / 2
        v = np.array([[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)]) * h + b.center
        faces = [[v[i] for i in f] for f in ([0, 1, 3, 2], [4, 5, 7, 6], [0, 1, 5, 4], [2, 3, 7, 6], [0, 2, 6, 4], [1, 3, 7, 5])]
        alpha = {'maze': 0.10, 'soffit': 0.25, 'step': 0.5}.get(b.kind, 0.85)
        ax.add_collection3d(Poly3DCollection(faces, facecolors=COLORS[b.kind], edgecolors='none' if b.kind == 'maze' else '#333333',
                                             linewidths=0.2, alpha=alpha))
    ax.scatter(*course.start, marker='*', s=160, color='#e0218a')
    ax.set_xlim(0, L)
    ax.set_ylim(0, W)
    ax.set_zlim(0, H)
    if hasattr(ax, 'set_box_aspect'):
        ax.set_box_aspect((L, W, H))
    ax.view_init(elev=35, azim=-60)
    ax.set_title('3D (walls faint)', fontsize=10)
    ax.tick_params(labelsize=7)
    st = course.stats()
    mz = st['maze']
    counts = ', '.join('%d %s' % (v, k + ('s' if v != 1 else '')) for k, v in st['obstacles'].items() if v)
    fig.suptitle('maze %d (%s split), %.1f x %.1f m, %.1f m tall, %dx%d cells, %d chambers (%d sunken, %d raised), %d loops, '
                 '%d dead ends; start on the %s; %s' % (course.seed, split_of(course.seed), L, W, H, *mz['grid'], mz['rooms'],
                                                        mz['sunken'], mz['raised'], mz['loops'], mz['dead_ends'],
                                                        course.params.start, counts), fontsize=10.5)
    fig.tight_layout(rect=(0, 0, 1, 0.92))
    fig.savefig(path, dpi=90)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--out', required=True)
    ap.add_argument('--split', choices=sorted(SPLITS), default='test')
    ap.add_argument('--count', type=int, default=10, help='first COUNT seeds of --split (ignored with --seeds)')
    ap.add_argument('--seeds', type=int, nargs='*')
    ap.add_argument('--start', choices=['floor', 'ceiling', 'both'], default='both')
    ap.add_argument('--difficulty', type=float, default=0.5)
    ap.add_argument('--layout', choices=['room', 'maze'], default='room')
    ap.add_argument('--no-preview', action='store_true')
    a = ap.parse_args()

    seeds = a.seeds if a.seeds else list(SPLITS[a.split][:a.count])  # range slicing: the train split is huge
    starts = ['floor', 'ceiling'] if a.start == 'both' else [a.start]
    out = os.path.expanduser(a.out)
    os.makedirs(out, exist_ok=True)
    index = []
    for seed in seeds:
        for start in starts:
            tic = time.time()
            course = generate(seed, CourseParams(start=start, difficulty=a.difficulty, layout=a.layout))
            folder = os.path.join(out, '%s_%d_%s' % ('course' if a.layout == 'room' else 'maze', seed, start))
            course.save(folder)
            if not a.no_preview:
                preview(course, os.path.join(folder, 'preview.png'))
            st = course.stats()
            index.append(dict(folder=os.path.basename(folder), split=split_of(seed), **st))
            print('%-24s %5.1f s  room %s  %s  explorable %.0f m3' % (os.path.basename(folder), time.time() - tic,
                  'x'.join('%.1f' % v for v in st['room_m']), st['obstacles'], st['explorable_m3']), flush=True)
    # merge with an existing index, so several runs into one --out keep every world listed
    path = os.path.join(out, 'index.json')
    if os.path.exists(path):
        with open(path) as f:
            mine = {r['folder'] for r in index}
            index = [r for r in json.load(f) if r.get('folder') not in mine] + index
    index.sort(key=lambda r: r['folder'])
    with open(path, 'w') as f:
        json.dump(index, f, indent=1)


if __name__ == '__main__':
    main()
