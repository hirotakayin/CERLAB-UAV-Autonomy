"""Vectorized sensor and map for training and for replaying Gazebo flights (stage 1).

BoxScene      ray hits against the SDF collision boxes, all rays at once. Exact surfaces (no voxel rounding, so thin
              boxes never disappear), the same thing Gazebo's lidar ray-traces.
traverse      the voxels each ray crosses, all rays at once (Amanatides-Woo stepping, numpy).
OccupancyMap  a port of map_manager's occMap update (log-odds, 20 m raycast length, 10 m inspect range), plus the
              coverage numbers epic_monitor_node logs (pit, block, floor, surface %), so a flight replayed here can be
              scored the same way as in Gazebo.
mid360_rays   Mid-360 ray directions: the recorded Livox pattern, or an even grid over the same field of view.

world.py keeps the one-ray-at-a-time reference ray-cast; this module is what training uses.
"""

import math

import numpy as np

from .world import load_sdf_boxes


class BoxScene:
    """Collision boxes as arrays: centre (M,3), rotation (M,3,3) world-from-box, half size (M,3)."""

    def __init__(self, boxes):
        self.names = [b.name for b in boxes]
        self.centre = np.array([b.translation for b in boxes], dtype=np.float64)
        self.rotation = np.array([b.rotation for b in boxes], dtype=np.float64)
        self.half = np.array([b.size * 0.5 for b in boxes], dtype=np.float64)
        self.radius = np.linalg.norm(self.half, axis=1)

    @classmethod
    def from_sdf(cls, path):
        return cls(load_sdf_boxes(path))

    def raycast(self, origin, directions, max_range):
        """Distance along each unit direction to the first box surface, inf when nothing is hit within max_range.

        origin (3,), directions (N,3). A ray starting inside a box hits at 0.
        """
        origin = np.asarray(origin, dtype=np.float64)
        directions = np.asarray(directions, dtype=np.float64)
        best = np.full(len(directions), np.inf)
        near = np.linalg.norm(self.centre - origin, axis=1) - self.radius <= max_range
        with np.errstate(divide='ignore', invalid='ignore'):
            for m in np.flatnonzero(near):
                rot, half = self.rotation[m], self.half[m]
                o = rot.T @ (origin - self.centre[m])  # ray in the box frame
                d = directions @ rot
                inv = 1.0 / d
                t1 = (-half - o) * inv
                t2 = (half - o) * inv
                # a ray parallel to a slab: inside it never limits t, outside it never hits (nan from 0 * inf)
                parallel = d == 0.0
                inside = np.abs(o) <= half
                lo = np.where(parallel, np.where(inside, -np.inf, np.inf), np.minimum(t1, t2))
                hi = np.where(parallel, np.where(inside, np.inf, -np.inf), np.maximum(t1, t2))
                t_near = lo.max(axis=1)
                t_far = hi.min(axis=1)
                hit = (t_near <= t_far) & (t_far >= 0.0)
                t = np.where(hit, np.maximum(t_near, 0.0), np.inf)
                np.minimum(best, t, out=best)
        best[best > max_range] = np.inf
        return best


def traverse(start, ends, grid_origin, resolution, shape_xyz):
    """Voxels crossed going from start to each end: the end voxel included, the start voxel not.

    start (3,), ends (N,3) in metres. Returns (ray, x, y, z) index arrays of every crossed voxel inside the grid.
    The number of steps to the end voxel is the L1 distance in voxels, so the rays are sorted by it and the still
    running ones are always a prefix: each step slices instead of masking.
    """
    res = float(resolution)
    p0 = (np.asarray(start, dtype=np.float64) - grid_origin) / res
    p1 = (np.asarray(ends, dtype=np.float64) - grid_origin) / res
    d = p1 - p0
    i0 = np.floor(p0).astype(np.int64)
    steps = np.abs(np.floor(p1).astype(np.int64) - i0).sum(axis=1)
    order = np.argsort(-steps, kind='stable')
    d, steps = d[order], steps[order]
    sign = np.sign(d).astype(np.int64)
    with np.errstate(divide='ignore', invalid='ignore'):
        inv = np.abs(1.0 / d)
        frac = p0 - i0
        t_max = np.where(d > 0, (1.0 - frac) * inv, np.where(d < 0, frac * inv, np.inf))
    t_delta = np.where(d != 0, inv, np.inf)
    idx = np.broadcast_to(i0, d.shape).copy()
    alive = len(d) - np.cumsum(np.bincount(steps, minlength=1))  # alive[k] = rays with more than k steps
    out_ray, out_idx = [], []
    rows = np.arange(len(d))
    for k in range(int(steps.max()) if len(d) else 0):
        n = alive[k]
        tm = t_max[:n]
        axis = tm.argmin(axis=1)
        r = rows[:n]
        idx[r, axis] += sign[r, axis]
        tm[r, axis] += t_delta[r, axis]
        out_ray.append(r)
        out_idx.append(idx[:n].copy())
    if not out_ray:
        empty = np.zeros(0, dtype=np.int64)
        return empty, empty, empty, empty
    ray = order[np.concatenate(out_ray)]
    cells = np.concatenate(out_idx)
    keep = np.all((cells >= 0) & (cells < shape_xyz), axis=1)
    return ray[keep], cells[keep, 0], cells[keep, 1], cells[keep, 2]


def _logit(p):
    return math.log(p / (1.0 - p))


class OccupancyMap:
    """map_manager's occMap, the parts the scoring uses, at one sensor pose per integrate() call.

    Arrays are indexed [x, y, z]. Matches occMap: points closer than 0.5 m are dropped; points outside the map or
    beyond raycast_max_length are pulled back and only clear space; a voxel that collects at least as many hits as
    misses in one scan gets p_hit, otherwise p_miss; unknown voxels a miss reaches become free at p_min; a surface
    hit from within inspect_range is inspected, one hit only from farther stays pending until a closer ray passes.
    Approximation: occMap's per-scan hit/miss tally depends on ray order (it stops a ray at a voxel an earlier ray
    already crossed); here a voxel's misses are 1 if it ended any ray plus 1 if any other ray crossed it.
    """

    UNKNOWN_FLAG = 0.01

    def __init__(self, origin, size, resolution, raycast_max_length=20.0, inspect_range=10.0,
                 p_hit=0.70, p_miss=0.35, p_min=0.12, p_max=0.97, p_occ=0.80):
        self.origin = np.asarray(origin, dtype=np.float64)
        self.size = np.asarray(size, dtype=np.float64)
        self.res = float(resolution)
        self.shape = np.ceil(self.size / self.res - 1e-9).astype(np.int64)
        self.max_len = float(raycast_max_length)
        self.inspect_range = float(inspect_range)
        self.hit_log, self.miss_log = _logit(p_hit), _logit(p_miss)
        self.min_log, self.max_log, self.occ_log = _logit(p_min), _logit(p_max), _logit(p_occ)
        self.occ = np.full(tuple(self.shape), self.min_log - self.UNKNOWN_FLAG, dtype=np.float64)
        self.inspected = np.zeros(tuple(self.shape), dtype=np.uint8)
        self.scans = 0

    # --- queries, as in occupancyMap.h
    def index(self, points):
        return np.floor((np.asarray(points, dtype=np.float64) - self.origin) / self.res).astype(np.int64)

    def in_map(self, idx):
        return np.all((idx >= 0) & (idx < self.shape), axis=-1)

    def known(self):
        return self.occ >= self.min_log

    def occupied(self):
        return self.occ >= self.occ_log

    # --- update
    def integrate(self, sensor, points):
        """One scan: sensor position (3,), return points in the world frame (N,3); rows with nan are no-returns."""
        sensor = np.asarray(sensor, dtype=np.float64)
        if not self.in_map(self.index(sensor)[None])[0]:
            return
        pts = np.asarray(points, dtype=np.float64)
        pts = pts[np.all(np.isfinite(pts), axis=1)]
        rel = pts - sensor
        dist = np.linalg.norm(rel, axis=1)
        pts, rel, dist = pts[dist >= 0.5], rel[dist >= 0.5], dist[dist >= 0.5]
        if len(pts) == 0:
            return
        self.scans += 1

        # outside the map: back along the ray to just inside it (occMap::adjustPointInMap), then cap the length
        lo, hi = self.origin + 1e-6, self.origin + self.size - 1e-6
        with np.errstate(divide='ignore', invalid='ignore'):
            t_lo, t_hi = (lo - sensor) / rel, (hi - sensor) / rel
            t_exit = np.where(rel != 0, np.maximum(t_lo, t_hi), np.inf).min(axis=1)
        adjusted = t_exit < 1.0
        scale = np.minimum(1.0, t_exit)
        long = dist * scale > self.max_len
        scale = np.where(long, self.max_len / dist, scale)
        adjusted |= long
        ends = sensor + rel * scale[:, None]

        hit_idx = self.index(ends[~adjusted])
        hit_idx = hit_idx[self.in_map(hit_idx)]
        hit_flat = np.ravel_multi_index(hit_idx.T, self.shape) if len(hit_idx) else np.zeros(0, dtype=np.int64)

        ray, cx, cy, cz = traverse(sensor, ends, self.origin, self.res, self.shape)
        crossed = np.ravel_multi_index((cx, cy, cz), self.shape)
        end_flat = np.ravel_multi_index(np.clip(self.index(ends), 0, self.shape - 1).T, self.shape)
        through = crossed[crossed != end_flat[ray]]  # crossed on the way to another voxel

        # per-voxel tallies for this scan, over the touched voxels only (sorted unique + searchsorted)
        touched = np.unique(np.concatenate((hit_flat, crossed)))
        h = np.zeros(len(touched), dtype=np.int64)
        if len(hit_flat):
            u, c = np.unique(hit_flat, return_counts=True)
            h[np.searchsorted(touched, u)] = c
        member = lambda keys: np.isin(touched, keys, assume_unique=False)
        misses = member(np.unique(end_flat)).astype(np.int64) + member(np.unique(through)).astype(np.int64)
        is_hit = (h >= misses) & (h > 0)

        flat = self.occ.reshape(-1)
        cur = flat[touched]
        upd = np.where(is_hit, self.hit_log, self.miss_log)
        new = np.clip(cur + upd, self.min_log, self.max_log)
        new = np.where(~is_hit & (cur < self.min_log), self.min_log, new)  # unknown, missed: free
        new = np.where(is_hit & (cur >= self.max_log), cur, new)  # clamped
        new = np.where(~is_hit & (cur == self.min_log), cur, new)
        flat[touched] = new

        if self.inspect_range > 0.0:
            ins = self.inspected.reshape(-1)
            close = np.linalg.norm(ends[~adjusted] - sensor, axis=1) <= self.inspect_range
            close = close[self.in_map(self.index(ends[~adjusted]))]
            if len(hit_flat):
                ins[hit_flat[close]] |= 2
                ins[hit_flat[~close]] |= 4
            centres = self.origin + (np.stack((cx, cy, cz), axis=1) + 0.5) * self.res
            near = np.linalg.norm(centres - sensor, axis=1) <= self.inspect_range
            ins[crossed[near]] &= np.uint8(~np.uint8(4))


class PitScore:
    """epic_monitor_node's coverage numbers for one region of an OccupancyMap, with the monitor's sampling grids."""

    def __init__(self, omap, region_min, region_max, floor_z=None):
        self.m = omap
        res = omap.res
        rmin, rmax = np.asarray(region_min, float), np.asarray(region_max, float)

        def grid(start, stop, step):  # the monitor's `for (v = start; v < stop; v += step)`, accumulated the same way
            out, v = [], start
            while v < stop:
                out.append(v)
                v += step
            return np.array(out)

        def flat(points):
            idx = omap.index(points)
            ok = omap.in_map(idx)
            out = np.full(len(points), -1, dtype=np.int64)
            out[ok] = np.ravel_multi_index(idx[ok].T, omap.shape)
            return out

        mesh = lambda *axes: np.stack(np.meshgrid(*axes, indexing='ij'), axis=-1).reshape(-1, len(axes))
        # pit %: voxel centres of the region
        self.pit = flat(mesh(*(grid(rmin[a] + res / 2, rmax[a], res) for a in range(3))))
        self.pit = self.pit[self.pit >= 0]
        # block %: 3x3-voxel blocks, each looked up at 27 points one voxel apart
        block = 3.0 * res
        centres = mesh(*(grid(rmin[a] + block / 2, rmax[a], block) for a in range(3)))
        offsets = mesh(*([np.array([-res, 0.0, res])] * 3))
        self.block = flat((centres[:, None, :] + offsets[None]).reshape(-1, 3)).reshape(len(centres), 27)
        # surface %: voxel corners of the region, from one block below it
        self.surface = flat(mesh(grid(rmin[0], rmax[0], res), grid(rmin[1], rmax[1], res),
                                 grid(rmin[2] - block, rmax[2], res)))
        # floor %: region columns at cell centres, floor_z +- 0.3
        self.floor = None
        if floor_z is not None:
            cols = int(math.ceil((rmax[0] - rmin[0]) / res))
            rows = int(math.ceil((rmax[1] - rmin[1]) / res))
            xs, ys = rmin[0] + (np.arange(cols) + 0.5) * res, rmin[1] + (np.arange(rows) + 0.5) * res
            zs = grid(floor_z - 0.3, floor_z + 0.3 + 1e-12, res)
            self.floor = flat(mesh(xs, ys, zs)).reshape(cols * rows, len(zs))
            self.floor_seen = np.zeros(cols * rows, dtype=bool)

    def __call__(self):
        known = self.m.known().reshape(-1)
        ins = self.m.inspected.reshape(-1)
        pending = (ins & 6) == 4
        out = {'pit_pct': 100.0 * known[self.pit].mean()}
        b = self.block
        valid = b >= 0
        any_known = (known[np.where(valid, b, 0)] & valid).any(axis=1)
        any_pending = (pending[np.where(valid, b, 0)] & valid).any(axis=1)
        out['pit_block_pct'] = 100.0 * (any_known & ~any_pending).mean()
        s = self.surface[self.surface >= 0]
        confirmed, pend = int(((ins[s] & 2) != 0).sum()), int(pending[s].sum())
        out['surface_pct'] = 100.0 * confirmed / (confirmed + pend) if confirmed + pend else float('nan')
        if self.floor is not None:
            f = self.floor
            ok = f >= 0
            self.floor_seen |= (((ins[np.where(ok, f, 0)] & 2) != 0) & ok).any(axis=1)
            out['floor_pct'] = 100.0 * self.floor_seen.mean()
        return out


def mid360_rays(pattern_csv=None, count=None, fov_down_deg=-7.0, fov_up_deg=52.0, azimuth_steps=None):
    """Unit ray directions in the lidar frame.

    pattern_csv: the recorded Livox pattern (the first `count` rows), what the Gazebo plugin fires.
    Otherwise an even grid over the Mid-360's field of view (360 deg x [fov_down, fov_up]), about `count` rays,
    spaced the same in azimuth and elevation, for training.
    """
    if pattern_csv is not None:
        from .world import load_mid360_directions
        return load_mid360_directions(pattern_csv, downsample=1, samples=count)
    count = count or 2000
    span = math.radians(fov_up_deg - fov_down_deg)
    if azimuth_steps is None:
        azimuth_steps = int(round(math.sqrt(count * 2 * math.pi / span)))
    elev_steps = max(1, int(round(count / azimuth_steps)))
    az = (np.arange(azimuth_steps) + 0.5) * 2 * math.pi / azimuth_steps
    el = np.radians(fov_down_deg) + (np.arange(elev_steps) + 0.5) * span / elev_steps
    a, e = np.meshgrid(az, el, indexing='ij')
    return np.stack((np.cos(e) * np.cos(a), np.cos(e) * np.sin(a), np.sin(e)), axis=-1).reshape(-1, 3)
