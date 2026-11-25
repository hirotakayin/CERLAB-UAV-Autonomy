#!/usr/bin/env python3
"""Figures of the one-node-per-column ("collapse") graph, drawn from the same simulator as prune_study.py.

Runs the greedy explorer on the collapse graph for --step steps in one ari3d world, freezes it, and draws:
  1. top view: the known map, and per column the one node kept (filled, coloured by its height) or the transit node
     kept where nothing in the column sees frontiers (hollow)
  2. side slice through one row of columns: every candidate height in each column (grey), the one kept (red =
     best utility, blue = transit), frontier cells near the row (orange), the drone, and the true walls
  3. the route to the node the explorer picks next: planned on the full 3D lattice, so it climbs and drops
     through any height on the way, even though the target column offered one height
  4. the same row --later steps later: the kept height of a column moves as its frontiers get seen

  column_graph_figure.py --world ~/mission_runs/ari3d_v3/img_1315_s2 --step 40 --out docs/column_graph.png
"""
import argparse
import os
import sys
from types import SimpleNamespace

import numpy as np
from scipy.sparse.csgraph import dijkstra

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from prune_study import MIN_UTILITY, Sim, nearest_node  # noqa: E402


def step_explorer(sim, p, visited, a):
    """One scan + one greedy move on the collapse set. Returns (new position, dist, k, pred, here, sets, score)."""
    sim.scan(p)
    sim.update(p)
    here = nearest_node(sim, p)
    d, pred = dijkstra(sim.graph, indices=here, return_predecessors=True)
    visited[here] = True
    score = sim.util / (d + a.xy)
    score[(sim.util < MIN_UTILITY) | ~np.isfinite(d) | visited] = -1
    S = sim.sets(p)
    cand = S['collapse']
    k = cand[np.argmax(score[cand])] if len(cand) else None
    return k, d, pred, here, S, score


def path_nodes(pred, here, k):
    out = [k]
    while out[-1] != here and pred[out[-1]] >= 0:
        out.append(pred[out[-1]])
    return out[::-1]


def draw_slice(ax, sim, X, yv, title):
    res = sim.res
    nz, ny, nx = sim.gt.shape
    yi = int(yv // res)
    img = np.where(sim.gt[:, yi, :], 0.15, np.where(X.known[:, yi, :] == 1, 0.97, 0.80))  # wall, known free, unknown
    ax.imshow(img, origin='lower', cmap='gray', vmin=0, vmax=1, extent=(0, nx * res, 0, nz * res), aspect='auto',
              interpolation='nearest')
    in_row = np.nonzero(X.valid & (np.abs(sim.lat_pos[:, 1] - yv) < 1e-6))[0]
    ax.scatter(sim.lat_pos[in_row, 0], sim.lat_pos[in_row, 2], s=10, c='#8a8f99', zorder=3, label='candidate heights (full graph)')
    kept = set(X.kept.tolist())
    uk = [i for i in in_row if i in kept and X.util[i] >= MIN_UTILITY]
    tk = [i for i in in_row if i in kept and X.util[i] < MIN_UTILITY]
    ax.scatter(sim.lat_pos[uk, 0], sim.lat_pos[uk, 2], s=70, c='#d1342b', edgecolors='white', linewidths=0.8,
               zorder=5, label='kept: most frontiers in its column')
    ax.scatter(sim.lat_pos[tk, 0], sim.lat_pos[tk, 2], s=55, facecolors='white', edgecolors='#2f6fd0',
               linewidths=1.6, zorder=5, label='kept: transit (no frontiers in column)')
    near = np.abs(X.front[:, 1] - yv) < 1.2
    ax.scatter(X.front[near, 0], X.front[near, 2], s=8, marker='s', c='#f08c00', zorder=4,
               label='frontier cells within 1.2 m of the row')
    if abs(X.p[1] - yv) < 2.5:
        ax.plot(X.p[0], X.p[2], marker='*', ms=16, color='#e0218a', mec='white', mew=0.8, zorder=6, label='drone')
    ax.set_title(title, fontsize=10)
    ax.set_xlabel('x (m)', fontsize=9)
    ax.set_ylabel('z (m)', fontsize=9)
    ax.tick_params(labelsize=8)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--world', required=True)
    ap.add_argument('--step', type=int, default=40)
    ap.add_argument('--later', type=int, default=12)
    ap.add_argument('--out', required=True)
    a0 = ap.parse_args()
    a = SimpleNamespace(range=16.0, xy=2.4, z=1.6, piece=5.0)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    sim = Sim(os.path.expanduser(a0.world), a)
    p = sim.start.astype(float)
    visited = np.zeros(len(sim.lat_idx), bool)
    traj = [p.copy()]
    for _ in range(a0.step):
        k, d, pred, here, S, score = step_explorer(sim, p, visited, a)
        if k is None or score[k] <= 0:
            break
        p = sim.lat_pos[k].copy()
        traj.append(p.copy())
    # freeze A: scan and plan once more, without moving
    k, d, pred, here, S, score = step_explorer(sim, p, visited, a)
    A = snapshot(sim, S, p)
    res = sim.res
    nz, ny, nx = sim.gt.shape
    nzl, nyl, nxl = sim.lat_shape

    # an example route that changes level: among kept nodes with frontiers, reachable, the one whose route climbs or
    # drops the most (ties: shorter route)
    kept = S['collapse']
    good = [i for i in kept if sim.util[i] >= MIN_UTILITY and np.isfinite(d[i]) and d[i] < 45]
    def climb(i):
        z = sim.lat_pos[path_nodes(pred, here, i), 2]
        return z.max() - z.min()
    target = max(good, key=lambda i: (round(climb(i), 1), -d[i])) if good else k
    route = path_nodes(pred, here, target)

    # B: continue the greedy run for --later steps
    q = p.copy()
    kk = k
    for _ in range(a0.later):
        q = sim.lat_pos[kk].copy()
        k2, _, _, _, _, sc2 = step_explorer(sim, q, visited, a)
        if k2 is None or sc2[k2] <= 0:
            break
        kk = k2
    sim.scan(q)
    sim.update(q)
    B = snapshot(sim, sim.sets(q), q)

    # the lattice row where the most columns changed their kept height between A and B
    def kept_by_col(X):
        return {int(sim.col[i]): float(sim.lat_pos[i, 2]) for i in X.kept}
    ka, kb = kept_by_col(A), kept_by_col(B)
    changed = [c for c in ka if c in kb and abs(ka[c] - kb[c]) > 1e-6]
    rows = np.array([c // nxl for c in changed]) if changed else np.array([sim.col[k] // nxl])
    row_y = int(np.bincount(rows).argmax())
    yv = sim.lat_pos[(sim.col // nxl == row_y)][0, 1]

    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig = plt.figure(figsize=(17, 11.5))
    gs = fig.add_gridspec(2, 2, height_ratios=[1.15, 1])

    # 1. top view with the column graph
    ax = fig.add_subplot(gs[0, 0])
    foot = sim.expl.any(0)
    img = np.where(~foot, 0.15, np.where(A.known_foot, 0.97, 0.80))
    ax.imshow(img, origin='lower', cmap='gray', vmin=0, vmax=1, extent=(0, nx * res, 0, ny * res), interpolation='nearest')
    colmap = {int(sim.col[i]): i for i in A.kept}
    for c, i in colmap.items():                        # links between kept nodes of neighbouring columns
        cy, cx = divmod(c, nxl)
        for dy, dx in ((0, 1), (1, 0), (1, 1), (1, -1)):
            j = colmap.get((cy + dy) * nxl + cx + dx) if 0 <= cx + dx < nxl else None
            if j is not None:
                ax.plot(sim.lat_pos[[i, j], 0], sim.lat_pos[[i, j], 1], color='#b9a27a', lw=0.5, zorder=2)
    u = np.array([i for i in A.kept if A.util[i] >= MIN_UTILITY])
    t = np.array([i for i in A.kept if A.util[i] < MIN_UTILITY])
    sc = ax.scatter(sim.lat_pos[u, 0], sim.lat_pos[u, 1], c=sim.lat_pos[u, 2], cmap='viridis', s=26, zorder=4,
                    edgecolors='white', linewidths=0.4)
    if len(t):
        ax.scatter(sim.lat_pos[t, 0], sim.lat_pos[t, 1], s=18, facecolors='white', edgecolors='#2f6fd0', linewidths=0.9, zorder=3)
    tr = np.array(traj)
    ax.plot(tr[:, 0], tr[:, 1], color='#e0218a', lw=1.2, alpha=0.8, zorder=5)
    ax.plot(p[0], p[1], marker='*', ms=16, color='#e0218a', mec='white', mew=0.8, zorder=6)
    rp = sim.lat_pos[route]
    ax.plot(rp[:, 0], rp[:, 1], color='#d1342b', lw=2.4, zorder=6)
    ax.plot(rp[-1, 0], rp[-1, 1], marker='o', ms=11, mfc='none', mec='#d1342b', mew=2.2, zorder=7)
    ax.axhline(yv, color='#d1342b', lw=0.8, ls='--', zorder=2)
    cb = fig.colorbar(sc, ax=ax, fraction=0.035, pad=0.01)
    cb.set_label('height of the kept node (m)', fontsize=9)
    cb.ax.tick_params(labelsize=8)
    ax.set_title('1. top view after %d steps: one kept node per column, linked to its neighbours (tan lines)\n'
                 'filled = most frontiers (colour = height); white = transit; pink = flown; red = example route; '
                 'dashed = row in 2 and 4' % (len(traj) - 1), fontsize=10)
    ax.tick_params(labelsize=8)

    # 2. side slice now
    ax2 = fig.add_subplot(gs[0, 1])
    draw_slice(ax2, sim, A, yv, '2. side slice y = %.1f m, now: grey = every candidate height of each column, '
               'red = the one kept,\n placed where its column sees the most frontiers (orange)' % yv)
    ax2.legend(fontsize=7.5, loc='upper right', framealpha=0.9)

    # 3. example route unrolled
    ax3 = fig.add_subplot(gs[1, 0])
    seg = np.r_[0, np.cumsum(np.linalg.norm(np.diff(rp, axis=0), axis=1))]
    fl, ce = [], []
    for q3 in rp:
        ci = np.floor(q3 / res).astype(int)
        colv = sim.gt[:, ci[1], ci[0]]
        lo = hi = ci[2]
        while lo > 0 and not colv[lo - 1]:
            lo -= 1
        while hi < nz - 1 and not colv[hi + 1]:
            hi += 1
        fl.append(lo * res)
        ce.append((hi + 1) * res)
    ax3.fill_between(seg, 0, fl, step='mid', color='#3a3f48', alpha=0.85, label='floor under the route (true map)')
    ax3.fill_between(seg, ce, nz * res, step='mid', color='#7d8594', alpha=0.7, label='ceiling over the route')
    ax3.plot(seg, rp[:, 2], '-o', color='#d1342b', ms=4, lw=2, zorder=4)
    ax3.plot(0, rp[0, 2], marker='*', ms=16, color='#e0218a', mec='white', mew=0.8, zorder=5)
    ax3.plot(seg[-1], rp[-1, 2], marker='o', ms=12, mfc='none', mec='#d1342b', mew=2.2, zorder=5)
    ax3.set_xlim(-0.5, seg[-1] + 0.5)
    ax3.set_ylim(0, nz * res)
    ax3.set_xlabel('distance along the route (m)', fontsize=9)
    ax3.set_ylabel('z (m)', fontsize=9)
    ax3.tick_params(labelsize=8)
    ax3.legend(fontsize=8, loc='upper left')
    ax3.set_title('3. example route to a kept node on another level, unrolled (%.1f m, %d hops): planned on the full 3D\n'
                  'lattice, so it climbs and drops wherever the space needs; collapse limits only which point is the goal'
                  % (seg[-1], len(route) - 1), fontsize=10)

    # 4. same row later
    ax4 = fig.add_subplot(gs[1, 1])
    draw_slice(ax4, sim, B, yv, '4. the same row %d steps later: kept heights have moved where frontiers were seen;\n'
               'columns with nothing left keep a transit node at the drone\'s height' % a0.later)
    fig.suptitle('One node per column ("collapse"), %s: the policy chooses among columns; the height inside a column '
                 'is the one that sees the most frontiers' % os.path.basename(a0.world.rstrip('/')), fontsize=12)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    os.makedirs(os.path.dirname(os.path.abspath(a0.out)), exist_ok=True)
    fig.savefig(a0.out, dpi=80)
    print('wrote', a0.out, '| kept', len(A.kept), 'of', len(A.full), 'candidates; columns that moved height in the row:',
          int((rows == row_y).sum()), '| example route', len(route) - 1, 'hops, climbs %.1f m' % (rp[:, 2].max() - rp[:, 2].min()))


def snapshot(sim, S, p):
    """What the figure needs at one moment, copied so the run can continue."""
    return SimpleNamespace(known=sim.known.copy(), known_foot=(sim.known == 1).any(0), valid=sim.valid.copy(),
                           util=sim.util.copy(), front=sim.front.copy(), kept=S['collapse'].copy(), full=S['full'].copy(),
                           p=p.copy())


if __name__ == '__main__':
    main()
