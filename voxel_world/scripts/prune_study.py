#!/usr/bin/env python3
"""Compare ways of pruning the 3D node graph, on ari3d worlds, with a greedy explorer standing in for the policy.

Exploration is simulated the way ARiADNE trains: the drone teleports from node to node and scans at each one. The
sensor is the tilted Mid-360's elevation band (-30.5..75.5 deg) over every heading, range --range. Nodes are a 3D
lattice (--xy, --z spacing) on known free configuration space, joined to their 26 lattice neighbours when the
straight segment is free. A node's utility is the number of frontier cells (1.6 m downsampled) within 0.8 x range,
in the band and in line of sight through known free space. The greedy explorer moves to the node with the best
utility / (path length + xy).

Candidate sets (what a policy would choose among / encode):
  full     every lattice node
  slices   per lattice column, the free runs cut into pieces of at most --piece m, one node (the middle) per piece:
           static, from geometry only
  gated    nodes with utility >= MIN_UTILITY, plus one transit node per column with none (the one nearest the
           drone's height) so the graph still routes: pruning by what has been observed
  collapse one node per column, the best-utility height (transit node if none): the policy picks a column, the
           height is picked inside it (a factorised action)

Two measurements per world:
  1. along the full-graph greedy run: each set's size, and the best score in it relative to the full best
  2. closed loop: the greedy explorer restricted to each set (paths still on the full graph), coverage vs distance

  prune_study.py --worlds ~/mission_runs/ari3d_try/0 ~/mission_runs/ari3d_try/2 --out ~/mission_runs/prune_study
"""
import argparse
import json
import math
import os
import sys
import time

import numpy as np
from scipy import ndimage
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import dijkstra
from scipy.spatial import cKDTree

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src'))
from voxel_world.fast import traverse  # noqa: E402

MIN_UTILITY = 2
EL_LO, EL_HI = -30.5, 75.5
STRATEGIES = ('full', 'slices', 'gated', 'collapse')


def sensor_dirs(step_deg=2.5):
    az = np.radians(np.arange(0, 360, step_deg))
    el = np.radians(np.arange(EL_LO, EL_HI + 1e-9, step_deg))
    A, E = np.meshgrid(az, el)
    return np.stack([np.cos(E) * np.cos(A), np.cos(E) * np.sin(A), np.sin(E)], -1).reshape(-1, 3)


class Sim:
    def __init__(self, world, a):
        z = np.load(os.path.join(world, 'world.npz'))
        self.gt, self.expl, self.surf = z['occupied'], z['explorable'], z['surface']
        self.res, self.start = float(z['resolution']), z['start']
        self.shape_xyz = np.array(self.gt.shape[::-1])
        self.known = np.zeros(self.gt.shape, np.int8)          # 0 unknown, 1 free, 2 occupied
        self.R, self.a = a.range, a
        self.dirs = sensor_dirs()
        # lattice
        res = self.res
        sx, sz = int(round(a.xy / res)), int(round(a.z / res))
        nz, ny, nx = self.gt.shape
        xs, ys, zs = np.arange(sx // 2, nx, sx), np.arange(sx // 2, ny, sx), np.arange(1, nz, sz)
        Z, Y, X = np.meshgrid(zs, ys, xs, indexing='ij')
        self.lat_idx = np.stack([X.ravel(), Y.ravel(), Z.ravel()], 1)            # voxel index (x, y, z)
        self.lat_pos = (self.lat_idx + 0.5) * res
        self.lat_shape = (len(zs), len(ys), len(xs))
        self.col = (np.arange(len(self.lat_idx)) % (len(ys) * len(xs)))         # column id = (y, x) cell
        self.lz = np.arange(len(self.lat_idx)) // (len(ys) * len(xs))
        self.util = np.zeros(len(self.lat_idx))
        self.nbr = self._lattice_pairs()
        self.surface_total, self.expl_total = self.surf.sum(), self.expl.sum()

    def _lattice_pairs(self):
        nzl, nyl, nxl = self.lat_shape
        ids = np.arange(nzl * nyl * nxl).reshape(self.lat_shape)
        pairs = []
        for dz in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for dx in (-1, 0, 1):
                    if (dz, dy, dx) <= (0, 0, 0):
                        continue
                    a = ids[max(0, -dz):nzl - max(0, dz), max(0, -dy):nyl - max(0, dy), max(0, -dx):nxl - max(0, dx)]
                    b = ids[max(0, dz):nzl + min(0, dz), max(0, dy):nyl + min(0, dy), max(0, dx):nxl + min(0, dx)]
                    pairs.append(np.stack([a.ravel(), b.ravel()], 1))
        return np.concatenate(pairs)

    # -- sensing
    def scan(self, p):
        ends = p + self.dirs * self.R
        ray, x, y, z = traverse(p, ends, np.zeros(3), self.res, self.shape_xyz)
        e = np.arange(len(ray))
        occ = self.gt[z, y, x]
        hit = np.full(len(self.dirs), np.iinfo(np.int64).max)
        np.minimum.at(hit, ray[occ], e[occ])
        keep = e <= hit[ray]
        self.known[z[keep & ~occ], y[keep & ~occ], x[keep & ~occ]] = 1
        self.known[z[keep & occ], y[keep & occ], x[keep & occ]] = 2
        i = np.floor(p / self.res).astype(int)
        self.known[i[2], i[1], i[0]] = 1

    def coverage(self):
        return float(((self.known == 1) & self.expl).sum() / self.expl_total), float(((self.known == 2) & self.surf).sum() / self.surface_total)

    # -- graph on the current belief
    def update(self, p):
        blocked = ndimage.binary_dilation(self.known != 1, structure=np.ones((3, 3, 3), bool))
        self.cfree = ~blocked
        li = self.lat_idx
        valid = self.cfree[li[:, 2], li[:, 1], li[:, 0]]
        self.valid = valid
        a, b = self.nbr[:, 0], self.nbr[:, 1]
        m = valid[a] & valid[b]
        a, b = a[m], b[m]
        pa, pb = self.lat_pos[a], self.lat_pos[b]
        ok = np.ones(len(a), bool)
        for t in np.linspace(0.2, 0.8, 4):
            q = np.floor((pa + t * (pb - pa)) / self.res).astype(int)
            ok &= self.cfree[q[:, 2], q[:, 1], q[:, 0]]
        a, b = a[ok], b[ok]
        w = np.linalg.norm(self.lat_pos[a] - self.lat_pos[b], axis=1)
        n = len(li)
        self.graph = csr_matrix((np.concatenate([w, w]), (np.concatenate([a, b]), np.concatenate([b, a]))), shape=(n, n))
        # frontiers (known free next to unknown), 1.6 m downsampled
        unk = self.known == 0
        nb = np.zeros_like(unk)
        nb[1:] |= unk[:-1]; nb[:-1] |= unk[1:]; nb[:, 1:] |= unk[:, :-1]; nb[:, :-1] |= unk[:, 1:]
        nb[:, :, 1:] |= unk[:, :, :-1]; nb[:, :, :-1] |= unk[:, :, 1:]
        fz, fy, fx = np.nonzero((self.known == 1) & nb)
        key = (fz // 4) * 1_000_000 + (fy // 4) * 1000 + (fx // 4)
        _, first = np.unique(key, return_index=True)
        self.front = (np.stack([fx[first], fy[first], fz[first]], 1) + 0.5) * self.res
        # utility, recomputed near the drone (farther nodes cannot have changed)
        ur = 0.8 * self.R
        near = np.nonzero(valid & (np.linalg.norm(self.lat_pos - p, axis=1) <= ur + self.R + 1.0))[0]
        self.util[~valid] = 0
        self.util[near] = 0
        if len(self.front) and len(near):
            tree = cKDTree(self.front)
            lists = tree.query_ball_point(self.lat_pos[near], ur)
            ni = np.repeat(near, [len(l) for l in lists])
            fi = np.concatenate([np.asarray(l, int) for l in lists]) if len(ni) else np.zeros(0, int)
            if len(ni):
                v = self.front[fi] - self.lat_pos[ni]
                d = np.linalg.norm(v, axis=1)
                el = np.degrees(np.arcsin(np.clip(v[:, 2] / np.maximum(d, 1e-9), -1, 1)))
                inb = (el >= EL_LO) & (el <= EL_HI)
                ni, fi = ni[inb], fi[inb]
                visible = self._los(ni, fi)
                np.add.at(self.util, ni[visible], 1)

    def _los(self, ni, fi):
        """Line of sight from each lattice node to each frontier through known-free cells (frontier cell excluded)."""
        out = np.ones(len(ni), bool)
        for s in range(0, len(ni), 200_000):
            sl = slice(s, s + 200_000)
            starts, ends = self.lat_pos[ni[sl]], self.front[fi[sl]]
            # traverse() takes one start; group by node
            order = np.argsort(ni[sl], kind='stable')
            nn = ni[sl][order]
            cuts = np.nonzero(np.diff(nn))[0] + 1
            for grp in np.split(np.arange(len(nn)), cuts):
                rows = order[grp]
                ray, x, y, z = traverse(starts[rows[0]], ends[rows], np.zeros(3), self.res, self.shape_xyz)
                bad = self.known[z, y, x] != 1
                endcell = np.floor(ends[rows] / self.res).astype(int)
                at_end = (x == endcell[ray, 0]) & (y == endcell[ray, 1]) & (z == endcell[ray, 2])
                blockedray = np.zeros(len(rows), bool)
                blockedray[ray[bad & ~at_end]] = True
                out[s + rows] = ~blockedray
        return out

    # -- candidate sets
    def sets(self, p):
        valid = np.nonzero(self.valid)[0]
        cols = self.col[valid]
        out = {'full': valid}
        # slices: runs of valid lattice nodes in a column (consecutive lz), cut into pieces of <= piece m
        per = max(1, int(self.a.piece / self.a.z))
        order = np.lexsort((self.lz[valid], cols))
        v = valid[order]
        c, lz = self.col[v], self.lz[v]
        newrun = np.ones(len(v), bool)
        newrun[1:] = (c[1:] != c[:-1]) | (lz[1:] != lz[:-1] + 1)
        run_id = np.cumsum(newrun) - 1
        run_start = np.nonzero(newrun)[0]
        pos_in_run = np.arange(len(v)) - run_start[run_id]
        run_len = np.bincount(run_id)
        piece = pos_in_run // per
        pieces_in_run = (run_len + per - 1) // per
        last_len = run_len - (pieces_in_run - 1) * per
        plen = np.where(piece == pieces_in_run[run_id] - 1, last_len[run_id], per)
        out['slices'] = v[(pos_in_run % per) == (plen - 1) // 2]
        # gated + transit
        good = valid[self.util[valid] >= MIN_UTILITY]
        has = np.zeros(self.col.max() + 1, bool)
        has[self.col[good]] = True
        rest = valid[~has[cols]]
        dz = np.abs(self.lat_pos[rest, 2] - p[2])
        o = np.lexsort((dz, self.col[rest]))
        rest = rest[o]
        first = np.ones(len(rest), bool)
        first[1:] = self.col[rest][1:] != self.col[rest][:-1]
        transit = rest[first]
        out['gated'] = np.concatenate([good, transit])
        # collapse: per column the best-utility node, transit if all zero
        o = np.lexsort((-self.util[valid], cols))
        vv = valid[o]
        first = np.ones(len(vv), bool)
        first[1:] = self.col[vv][1:] != self.col[vv][:-1]
        best = vv[first]
        best = best[self.util[best] >= MIN_UTILITY]
        out['collapse'] = np.concatenate([best, transit])
        return out


def nearest_node(sim, p):
    v = np.nonzero(sim.valid)[0]
    return v[np.argmin(np.linalg.norm(sim.lat_pos[v] - p, axis=1))]


def run(world, a, policy_set, log_sets=False):
    sim = Sim(world, a)
    p = sim.start.astype(float)
    dist_total, steps, curve, trace = 0.0, 0, [], []
    visited = np.zeros(len(sim.lat_idx), bool)
    t0 = time.time()
    while steps < a.max_steps:
        sim.scan(p)
        sim.update(p)
        cov = sim.coverage()
        curve.append((dist_total, *cov))
        here = nearest_node(sim, p)
        d = dijkstra(sim.graph, indices=here)
        score = sim.util / (d + a.xy)
        visited[here] = True
        score[(sim.util < MIN_UTILITY) | ~np.isfinite(d) | visited] = -1
        S = sim.sets(p)
        if log_sets:
            best_full = score[S['full']].max()
            rec = dict(step=steps, coverage=cov, columns=int(len(np.unique(sim.col[S['full']]))))
            for k in STRATEGIES:
                rec[k + '_n'] = int(len(S[k]))
                rec[k + '_ratio'] = float(max(0.0, score[S[k]].max()) / best_full) if best_full > 0 and len(S[k]) else None
            trace.append(rec)
        cand = S[policy_set]
        k = cand[np.argmax(score[cand])] if len(cand) else None
        if k is None or score[k] <= 0:
            break
        dist_total += d[k]
        p = sim.lat_pos[k].copy()
        steps += 1
    return dict(world=os.path.basename(world.rstrip('/')), policy=policy_set, steps=steps, distance=round(dist_total, 1),
                coverage=curve[-1][1:], curve=curve, trace=trace, seconds=round(time.time() - t0, 1))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--worlds', nargs='+', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--range', type=float, default=16.0)
    ap.add_argument('--xy', type=float, default=2.4)
    ap.add_argument('--z', type=float, default=1.6)
    ap.add_argument('--piece', type=float, default=5.0)
    ap.add_argument('--max-steps', type=int, default=400)
    ap.add_argument('--policies', nargs='+', default=list(STRATEGIES))
    a = ap.parse_args()
    out = os.path.expanduser(a.out)
    os.makedirs(out, exist_ok=True)
    for w in a.worlds:
        w = os.path.expanduser(w)
        for pol in a.policies:
            r = run(w, a, pol, log_sets=(pol == 'full'))
            name = '%s_%s.json' % (r['world'], pol)
            with open(os.path.join(out, name), 'w') as f:
                json.dump(r, f)
            msg = '%s %-8s steps %3d  distance %7.1f m  volume %.3f  surface %.3f  (%.0f s)' % (
                r['world'], pol, r['steps'], r['distance'], *r['coverage'], r['seconds'])
            if r['trace']:
                tr = r['trace']
                msg += '\n    mean set size: ' + ', '.join('%s %.0f' % (k, np.mean([t[k + '_n'] for t in tr])) for k in STRATEGIES)
                msg += '; columns %.0f' % np.mean([t['columns'] for t in tr])
                msg += '\n    best score kept (mean ratio / steps >= 0.95): ' + ', '.join(
                    '%s %.3f / %.0f%%' % (k, np.mean([t[k + '_ratio'] for t in tr if t[k + '_ratio'] is not None]),
                                          100 * np.mean([(t[k + '_ratio'] or 0) >= 0.95 for t in tr])) for k in STRATEGIES)
            print(msg, flush=True)


if __name__ == '__main__':
    main()
