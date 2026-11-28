"""3D training worlds from ARiADNE's 2D map PNGs: one map's topology, heights varied per region.

ARiADNE trains on 5,663 floor-plan PNGs (marmotlab/large-scale-DRL-exploration, maps/): 640 x 480 px at 0.4 m,
127 solid, 194 free, a 16 x 16 px block of 208 at the start. This module turns one map into a voxel world:

  1. free space -> regions: watershed of the distance to the walls, seeds at least region_size apart, so a
     region is a room or a piece of corridor
  2. each region gets a level, shifting its floor and ceiling together (a terrace), and its own clear height;
     a link between regions keeps the level or steps it up or down; some regions are pits (the floor drops,
     the ceiling stays)
  3. each solid island (not the outer boundary) is either full height, low enough to fly over, or full height
     with an overhanging lip that roofs the free space next to it (hiding the floor under it from above)
  4. solve: linked regions must share `passage` of height (the lower ceiling is lifted); while part of the
     free configuration space is cut off from the start, the region there is opened up to span its neighbours;
     then, while one constant flight height still reaches more than max_flat_reach of the footprint in one
     piece, a gate closes that height on a narrow link it passes through: a hurdle (open only at the top of
     the band the two regions share) or a lintel (open only at the bottom)

Clearance: every opening the drone has to use (between levels, under gates and lips, over low islands) is at
least `passage` tall in whole voxels (3.2 m by default), and the solver checks reachability with the drone's box
grown by `margin` (0.5 m) on every side. A world is `accepted` when no single height reaches more than
max_flat_reach of it and the grown box still reaches min_margin_reach of it.

Everything stays a height field per (x, y) column: free between floor(x, y) and ceiling(x, y), with island columns
free only above the island top. With all variation off (make_ari3d --flat), the result is the 2D map extruded, which gives a
check that a 3D trainer reproduces ARiADNE on its own maps.

The voxel z = 0 is the deepest floor; the start region's level is at `base`. No SDF export yet (the walls are not boxes).
"""

from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path

import numpy as np

FREE, SOLID, START = 194, 127, 208


@dataclass
class Ari3DParams:
    pixel: float = 0.2                  # m per PNG pixel: 0.2 is ARiADNE's own scale (its env.py halves the PNG to 0.4 m cells)
    resolution: float = 0.4             # m, voxel size
    region_size: float = 10.0           # m, least distance between region seeds
    # levels: a region's floor and ceiling move together (a terrace), so no one flight height is open everywhere
    level_change: float = 0.7           # share of region links where the level changes
    level_step: tuple = (2.0, 4.0)      # m, size of a level change (up or down, cut to what passage allows)
    level_keep: float = 0.75            # chance a change keeps the direction of the change before it (drift)
    level_range: tuple = (-10.0, 10.0)  # m, levels stay in this range around the start region's level
    # staggers: a shaft region whose neighbours on one side sit a whole room height (plus a slab) above the
    # other side, so the upper rooms' floors are above the lower rooms' ceilings; the shaft spans both
    stagger_chance: float = 0.6         # chance an eligible region becomes a shaft while the level walk passes it
    max_staggers: int = 3
    min_staggers: int = 1               # a world needs this many to be `accepted` (height draws are retried for it)
    stagger_slab: tuple = (0.8, 2.0)    # m, solid between the lower room's ceiling and the upper room's floor
    shaft_min_m2: float = 30.0          # m2, a shaft needs this much floor ...
    shaft_max_m2: float = 150.0         # m2, ... and at most this much, so it stays a connector, not a hall
    shaft_min_width: float = 3.0        # m, ... and to be at least this wide somewhere
    # bridges: where a corridor turns around the corner of a solid block, a second, enclosed route cuts across the top
    # of the block between the corridor's two legs. The lower route is the corridor with its L corner, the upper one
    # an L turning at the opposite corner of the same rectangle; they meet again at both ends (a loop in x-z / y-z)
    max_bridges: int = 2
    bridge_span: tuple = (8.0, 25.0)    # m, offset between the two ends, in both x and y
    bridge_width: tuple = (3.2, 4.4)    # m
    bridge_height: tuple = (4.0, 6.0)   # m, the enclosed passage's height
    bridge_rise: tuple = (3.2, 5.2)     # m, its floor above the higher of the two corridor floors at the ends
    bridge_tries: int = 600
    clear: tuple = (4.8, 7.2)           # m, a region's floor-to-ceiling height (leaves room for level steps above passage)
    sunken: tuple = (0.15, 3.0, 6.0)    # (share of regions, shallowest, deepest extra drop of the floor in m): pits
    low_island: tuple = (0.35, 1.5)     # (share of islands, least height above the floors around it in m)
    overhang: tuple = (0.25, 1.6, 4.0)  # (share of full islands, shortest, longest reach of the lip in m)
    # clearance: every opening the drone must use is at least `passage` tall (rounded up to whole voxels), and the
    # solver checks reachability with the drone's box grown by `margin` on every side, so nothing is a squeeze
    passage: float = 3.0                # m, least height of any opening: between levels, under gates and lips, over islands
    margin: float = 0.5                 # m, clearance around the drone's box that every route must allow
    min_margin_reach: float = 0.99      # share of the footprint reachable with the grown box, for `accepted`
    # height gates on narrow links, added until no single flight height reaches more than max_flat_reach
    gate_width: float = 14.0            # m, links at most this wide can get a gate
    gate_thick: tuple = (0.8, 1.6)      # m, the gate's thickness along the passage
    max_flat_reach: float = 0.6         # share of the footprint one constant height may reach in one piece
    max_gates: int = 16
    tries: int = 10                     # height draws per map; the one with the lowest single-height reach is kept
    min_region_m2: float = 25.0         # m2, smaller regions join the neighbour they share most border with
    min_island_m2: float = 4.0          # m2, smaller solid blobs (edge noise) stay full height
    robot_size: float = 1.0             # m, the drone's safety box
    start_clearance: float = 0.6        # m above the start region's floor
    # clutter in big open spaces (off by default, so older worlds reproduce). An open zone is the union of the widest
    # discs that fit (centres at least open_radius from walls); a zone of at least clutter_min_m2 gets, with
    # clutter_chance, one of the `clutter` kinds:
    #   'pillars'  a jittered grid of thin full-height pillars, joined by horizontal struts at one or two heights
    #              (excavation1: 0.4 m pillars 4 m x 6 m apart, one strut layer); the hall's regions get clutter_clear of
    #              height so the struts leave a passage above and below
    #   'lwalls'   full-height L-shaped walls, free-standing or growing out of a wall, that split the hall into bays
    #   'mixed'    L walls first, then a sparse pillar grid (+ struts) in the space they leave
    # Every gap left is at least clutter_gap (L walls) / pillar_gap wide (2D check: the free space eroded by half of it
    # keeps its pieces)
    clutter: tuple = ()                 # kinds to draw from per zone: () off, any of 'pillars', 'lwalls', 'mixed'
    clutter_weights: tuple = ()         # draw weights, one per kind in `clutter` (empty: equal)
    open_radius: float = 4.0            # m, an open zone holds discs at least this big (8 m across)
    clutter_min_m2: float = 250.0       # m2, least zone area that gets clutter
    clutter_chance: float = 0.8         # share of big zones that get clutter (some halls stay open)
    clutter_gap: float = 4.4            # m, least free width left between an L wall and anything solid. The planner's
                                        # lattice (4 m columns) needs this: at 2.8 m bays were cut off its graph
    pillar_gap: float = 3.2             # m, the same for a pillar (a free-standing pillar cannot close off a bay)
    clutter_clear: tuple = (8.0, 12.0)  # m, floor-to-ceiling height of a pillar hall's regions
    pillar_size: tuple = (0.4, 1.2)     # m, square pillar side
    pillar_spacing: tuple = (4.0, 7.0)  # m, grid pitch, drawn per axis
    pillar_jitter: float = 0.4          # m, random shift of each pillar off the grid
    pillar_skip: float = 0.1            # share of grid points left empty
    mixed_pillar_skip: float = 0.5      # the same in a 'mixed' zone (pillars between the L walls stay sparse)
    strut_chance: float = 0.6           # share of pillar-pillar (and pillar-wall) links that get a strut, per layer
    strut_layers: tuple = (1, 2)        # number of strut heights per hall
    strut_thick: float = 0.4            # m, a strut's depth (z) and width
    lwall_area: float = 60.0            # m2 of zone per L wall (tried; walls that do not fit are dropped)
    lwall_max: int = 12                 # L walls per zone at most
    lwall_leg: tuple = (5.0, 12.0)      # m, each leg's length
    lwall_thick: tuple = (0.4, 0.8)     # m
    lwall_attach: float = 0.6           # share of L walls that grow out of a wall (their first leg starts at it): bays


def load_png(path):
    """(free, image) from an ARiADNE map PNG: free [row, col] bool (as ARiADNE's env.py reads it) and the grey levels."""
    from skimage import io
    img = (io.imread(str(path), as_gray=True) * 255).astype(int)
    free = (img > 150) | ((img <= 80) & (img >= 50))
    return free, img


class Ari3D:
    """A voxel world built from one PNG. Grids are [z, y, x]; y grows with the PNG's row index."""

    def __init__(self, png, seed=0, params=None):
        p = params or Ari3DParams()
        best = None
        for trial in range(max(1, p.tries)):
            self._generate(png, seed, p, np.random.default_rng([seed, trial]))
            self.info['trial'] = trial
            key = lambda d: (d['margin_reach'] >= p.min_margin_reach, d['info']['staggers'] >= p.min_staggers,
                             -d['flat_reach'])
            if best is None or key(self.__dict__) > key(best):
                best = dict(self.__dict__)
            if self.flat_reach <= p.max_flat_reach and self.margin_reach >= p.min_margin_reach and \
                    self.info['staggers'] >= p.min_staggers:
                break
        self.__dict__.update(best)
        self.info['accepted'] = bool(self.flat_reach <= p.max_flat_reach and self.margin_reach >= p.min_margin_reach
                                     and self.info['staggers'] >= p.min_staggers
                                     and self.info['reachable_share'] >= 0.99)   # training should skip the rest

    def _generate(self, png, seed, p, rng):
        from scipy import ndimage
        self.params = p
        self.png, self.seed = str(png), seed
        free_px, img = load_png(png)

        # 2D free grid at the voxel size, as ARiADNE's env.py builds it: block minimum, so a cell is solid when any of
        # its pixels is; the start is ARiADNE's too (the 11th start-marker cell in row-major order)
        k = p.resolution / p.pixel
        if abs(k - round(k)) > 1e-6 or round(k) < 1:
            raise ValueError('resolution must be a whole multiple of pixel')
        k = int(round(k))
        h, w = (free_px.shape[0] // k) * k, (free_px.shape[1] // k) * k
        free = free_px[:h, :w].reshape(h // k, k, w // k, k).all(axis=(1, 3))
        img_k = img[:h, :w].reshape(h // k, k, w // k, k).min(axis=(1, 3))
        sr, sc = np.nonzero(img_k == START)
        if len(sr) == 0:
            raise ValueError('no start marker (208) left after downsampling')
        start_cell = (int(sr[min(10, len(sr) - 1)]), int(sc[min(10, len(sr) - 1)]))
        rows, cols = np.nonzero(free)
        r0, r1 = max(0, rows.min() - 2), min(free.shape[0], rows.max() + 3)
        c0, c1 = max(0, cols.min() - 2), min(free.shape[1], cols.max() + 3)
        free = free[r0:r1, c0:c1]
        start_rc = (start_cell[0] - r0, start_cell[1] - c0)
        free = _largest_containing(free, start_rc, ndimage)
        self.free2d = free
        res = p.resolution

        # 1. regions
        dist = ndimage.distance_transform_edt(free)
        size = max(3, int(round(p.region_size / res)) | 1)
        peaks = free & (dist == ndimage.maximum_filter(dist, size=size)) & (dist >= 1.5)
        markers, n_regions = ndimage.label(peaks)
        from skimage.segmentation import watershed
        regions = watershed(-dist, markers, mask=free).astype(np.int32)
        regions = _merge_small(regions, p.min_region_m2 / (res * res))
        # 1b. clutter in big open spaces: pillars (+ struts) and L walls become solid in the 2D map; the regions stay
        #     those of the open map (a pillar grid must not shatter a hall into many small terraces)
        self.clutter = dict(mask=np.zeros_like(free), struts=[], zones=[], pillars=0, lwalls=0, halls=[])
        if p.clutter:
            self.clutter = _plan_clutter(free, dist, start_rc, p, np.random.default_rng([seed, int(rng.integers(1 << 30)), 7]),
                                         ndimage)
            cm = self.clutter['mask']
            hall = self.clutter['hall_mask']
            self.clutter['halls'] = [int(i) for i in np.unique(regions[hall]) if i > 0
                                     and (hall & (regions == i)).sum() >= 0.5 * (regions == i).sum()]
            free = free & ~cm
            regions = np.where(cm, 0, regions)
            dist = ndimage.distance_transform_edt(free)
            self.free2d = free
        ids = [int(i) for i in np.unique(regions) if i > 0]
        self.regions = regions

        # 2. levels: walk the region graph from the start region; each link keeps or changes the level
        pairs = _touching_pairs(regions)
        adj = {i: set() for i in ids}
        for a_, b_ in pairs:
            adj[a_].add(b_)
            adj[b_].add(a_)
        snap = lambda v: round(v / res) * res
        gap = math.ceil(p.passage / res - 1e-9) * res        # the passage in whole voxels
        self.gap = gap
        clear = {i: snap(rng.uniform(*p.clear)) for i in ids}
        for i in self.clutter['halls']:                  # pillar halls are tall: struts with a passage above and below
            if i in clear:
                clear[i] = snap(rng.uniform(*p.clutter_clear))
        level = {}
        first = int(regions[start_rc]) if regions[start_rc] > 0 else ids[0]
        level[first], queue = 0.0, [first]
        heading = {first: 1.0 if rng.random() < 0.5 else -1.0}
        shafts = {}                                       # region -> (lower level, upper level)
        area = {i: float((regions == i).sum()) * res * res for i in ids}
        width = {i: 2 * float(dist[regions == i].max()) * res for i in ids}
        while queue:
            q = queue.pop(int(rng.integers(len(queue))))      # random order, so levels do not just follow BFS rings
            for c in sorted(adj[q]):
                if c in level:
                    continue
                onward = [n for n in adj[c] if n not in level and n != q]
                if (len(shafts) < p.max_staggers and q not in shafts and onward and p.shaft_min_m2 <= area[c] <= p.shaft_max_m2
                        and width[c] >= p.shaft_min_width and rng.random() < p.stagger_chance):
                    # c becomes a shaft: the rooms past it jump a whole room height (and a slab) up or down
                    up = heading[q] if rng.random() < p.level_keep else -heading[q]
                    for sgn in (up, -up):
                        jump = clear[q] + snap(rng.uniform(*p.stagger_slab)) if sgn > 0 else -(max(clear[n] for n in onward)
                                                                                           + snap(rng.uniform(*p.stagger_slab)))
                        if p.level_range[0] <= level[q] + jump <= p.level_range[1]:
                            break
                    else:
                        jump = None
                    if jump is not None:
                        level[c] = level[q]
                        heading[c] = math.copysign(1.0, jump)
                        shafts[c] = (min(level[q], level[q] + jump), max(level[q], level[q] + jump))
                        for n in onward:                  # the far side starts on the new level
                            level[n] = level[q] + jump
                            heading[n] = heading[c]
                            queue.append(n)
                        queue.append(c)
                        continue
                d = 0.0
                heading[c] = heading[q] if rng.random() < p.level_keep else -heading[q]
                if rng.random() < p.level_change:
                    room = min(clear[q], clear[c]) - gap
                    d = heading[c] * snap(min(rng.uniform(*p.level_step), room))
                    if not p.level_range[0] <= level[q] + d <= p.level_range[1]:
                        d, heading[c] = -d, -heading[c]          # bounce off the range
                level[c] = float(np.clip(level[q] + d, *p.level_range))
                queue.append(c)
        for i in ids:                                    # regions not linked to the start one (should not happen)
            level.setdefault(i, 0.0)
        floor = {i: level[i] for i in ids}
        ceil = {i: level[i] + clear[i] for i in ids}
        kind = {i: 'level' for i in ids}
        for c, (lo, hi) in shafts.items():                # a shaft spans from the lower floor to the upper ceiling
            floor[c] = lo
            ceil[c] = hi + max(clear[n] for n in adj[c])
            kind[c] = 'shaft'
        for i in ids:
            if kind[i] == 'shaft':
                continue
            if rng.random() < p.sunken[0]:
                kind[i] = 'sunken'
                floor[i] -= snap(rng.uniform(*p.sunken[1:]))

        # 3. islands: low (fly over), full, or full with an overhanging lip
        solid_lab, n_solid = ndimage.label(~free)
        border = set(np.unique(np.concatenate([solid_lab[0], solid_lab[-1], solid_lab[:, 0], solid_lab[:, -1]]))) - {0}
        islands = {}
        for j in range(1, n_solid + 1):
            if j in border:
                continue
            mask = solid_lab == j
            if mask.sum() * res * res < p.min_island_m2:
                continue
            ring = ndimage.binary_dilation(mask, iterations=1) & free
            around = [i for i in np.unique(regions[ring]) if i > 0]
            if not around:
                continue
            u = rng.random()
            if self.clutter['mask'][mask].any():
                islands[j] = dict(kind='full', around=around)    # clutter stays full height
            elif u < p.low_island[0]:
                islands[j] = dict(kind='low', around=around, frac=rng.uniform(0.2, 0.8))
            elif u < p.low_island[0] + (1 - p.low_island[0]) * p.overhang[0]:
                islands[j] = dict(kind='lip', around=around, reach=rng.uniform(*p.overhang[1:]), frac=rng.uniform(0.2, 0.8))
            else:
                islands[j] = dict(kind='full', around=around)

        # bridges over the inside corners of the corridors (after the islands: a bridge makes the blocks it crosses full)
        bridges = []
        if p.max_bridges > 0:
            bridges = _plan_bridges(free, regions, solid_lab, islands, rng, p, res, snap, gap, ndimage)
        decks = bridges                                   # the solver and _build treat them alike below

        # 4. solve: shared height between touching regions; reachability (flatten as the fallback); then height
        #    gates until no single flight height reaches more than max_flat_reach of the footprint
        bounds = {}
        for a_, b_ in pairs:
            m = _boundary(regions, a_, b_)
            if m.sum() * res / 2 <= p.gate_width:
                bounds[(a_, b_)] = m
        gates, self.flattened = [], []
        flat = None
        for attempt in range(len(ids) + p.max_gates + 2):
            for _ in range(4):
                for a_, b_ in pairs:
                    if _joined_by_shaft(a_, b_, adj, kind, floor, ceil, gap):
                        continue                          # a stagger: the two meet only through the shaft beside them
                    need = max(floor[a_], floor[b_]) + gap
                    for i in (a_, b_):
                        if ceil[i] < need:
                            ceil[i] = need
            occ, fmap, cmap, base = self._build(free, regions, solid_lab, islands, floor, ceil, gates, decks)
            start = self._start_point(start_rc, fmap, base)
            cfree = _cspace(occ, p.robot_size, res, ndimage)
            si = tuple(np.floor(start / res).astype(int)[::-1])
            if not cfree[si]:
                raise RuntimeError('start not in free configuration space')
            lab, _ = ndimage.label(cfree)
            reach = lab == lab[si]
            # the same with the box grown by margin: columns the drone reaches only by squeezing count as cut
            big = _cspace(occ, p.robot_size + 2 * p.margin, res, ndimage)
            col = np.nonzero(big[:, si[1], si[2]])[0]
            foot = reach.any(0)
            if len(col):
                lab_b, _ = ndimage.label(big)
                reach_b = (lab_b == lab_b[col[np.argmin(np.abs(col - si[0]))], si[1], si[2]]).any(0)
                # the grown box keeps `margin` further from walls: widen its footprint back by that much, so only
                # space behind a narrow opening counts as squeezed, not the strip along every wall
                w = int(math.ceil(p.margin / res))
                reach_b = ndimage.binary_dilation(reach_b, structure=np.ones((2 * w + 1, 2 * w + 1), bool))
            else:
                reach_b = np.zeros_like(foot)
            self.margin_reach = float((foot & reach_b).sum() / max(1, foot.sum()))
            squeezed = foot & ~reach_b
            cut = (cfree & ~reach) | (squeezed[None] & cfree)
            if (cfree & ~reach).sum() > 0.01 * cfree.sum() or squeezed.sum() > (1 - p.min_margin_reach) * foot.sum():
                if gates and gates[-1].get('fresh'):          # the last gate cut the map: drop it
                    gates.pop()
                    self.flattened.append(('gate', None))
                    continue
                zz, yy, xx = np.nonzero(cut)
                counts = np.bincount(regions[yy, xx], minlength=regions.max() + 1)
                counts[0] = 0
                worst = int(counts.argmax())
                isl = np.bincount(solid_lab[yy, xx], minlength=n_solid + 1)
                isl[0] = 0
                if counts[worst] == 0 and int(isl.argmax()) in islands and islands[int(isl.argmax())]['kind'] != 'full':
                    islands[int(isl.argmax())]['kind'] = 'full'   # cut-off space above a low island: make it full
                    self.flattened.append(('island', int(isl.argmax())))
                elif decks and (counts[worst] == 0 or ('region', worst) in self.flattened):
                    # opening the cut region did not help: drop the bridge nearest the cut-off space
                    cy, cx = np.mean(yy), np.mean(xx)
                    drop = min(range(len(decks)), key=lambda j: np.hypot(*(np.mean(np.nonzero(decks[j]['upper_band']),
                                                                                   axis=1) - (cy, cx))))
                    decks = decks[:drop] + decks[drop + 1:]
                    self.flattened.append(('bridge', drop))
                elif counts[worst] == 0:
                    break                # nothing left to change: keep this draw (not accepted)
                else:                    # open the region up to span its neighbours
                    floor[worst] = min(floor[n] for n in adj[worst]) if adj[worst] else floor[worst]
                    ceil[worst] = max(ceil[n] for n in adj[worst]) if adj[worst] else ceil[worst]
                    kind[worst] = 'level'
                    self.flattened.append(('region', worst))
                continue
            for g in gates:
                g.pop('fresh', None)
            flat, k, layer_lab = _flat_reach(cfree & reach)
            if flat <= p.max_flat_reach or len(gates) >= p.max_gates:
                break
            # close the dominant height on a narrow link it passes through
            zk = (k + 0.5) * res
            comp = layer_lab == np.bincount(layer_lab.ravel())[1:].argmax() + 1
            options = []
            for (a_, b_), m in bounds.items():
                if any(g['pair'] == (a_, b_) for g in gates) or not (m & comp).any():
                    continue
                lo, hi = max(floor[a_], floor[b_]) + base, min(ceil[a_], ceil[b_]) + base
                if hi - lo < gap + 2 * res:
                    continue
                if zk < hi - gap - 0.2:
                    options.append(((a_, b_), 'hurdle'))      # passage only at the top of the shared band
                elif zk > lo + gap + 0.2:
                    options.append(((a_, b_), 'lintel'))      # passage only at the bottom
            if not options:
                break
            pair, gk = options[int(rng.integers(len(options)))]
            t = max(1, int(round(rng.uniform(*p.gate_thick) / res / 2)))
            strip = ndimage.binary_dilation(bounds[pair], iterations=t) & ((regions == pair[0]) | (regions == pair[1]))
            gates.append(dict(pair=pair, kind=gk, strip=strip, fresh=True))
        else:                                   # out of attempts with part of the map still cut or squeezed
            pass
        if 'flat' not in locals() or flat is None:
            flat, k, _ = _flat_reach(cfree & reach)
        self.flat_reach, self.flat_height = flat, (k + 0.5) * res
        self.gates = gates
        top = max(ceil.values())
        self.occupied, self.floor_map, self.ceil_map, self.base, self.top = occ, fmap, cmap, base, top
        self.start = start
        self.explorable = _largest_free_from(~occ, si, ndimage)
        self.surface = occ & _dilate6(self.explorable)
        self.origin = np.zeros(3)
        self.kind = kind
        self.bridges = decks
        self.islands = islands
        self.info = dict(png=Path(self.png).name, seed=seed, size_m=[round(free.shape[1] * res, 1), round(free.shape[0] * res, 1),
                                                                      round(occ.shape[0] * res, 1)],
                         start_level_z_m=round(base, 2), regions=len(ids),
                         levels=len({round(floor[i], 1) for i in ids if kind[i] == 'level'}),
                         level_span_m=round(max(level.values()) - min(level.values()), 1),
                         sunken=sum(v == 'sunken' for v in kind.values()), islands=len(islands),
                         staggers=sum(v == 'shaft' for v in kind.values()), bridges=len(decks),
                         low_islands=sum(v['kind'] == 'low' for v in islands.values()),
                         lips=sum(v['kind'] == 'lip' for v in islands.values()),
                         gates=len(gates), hurdles=sum(g['kind'] == 'hurdle' for g in gates),
                         lintels=sum(g['kind'] == 'lintel' for g in gates),
                         flat_reach=round(float(self.flat_reach), 3), flat_height_m=round(float(self.flat_height), 1),
                         passage_m=round(self.gap, 2), margin_reach=round(self.margin_reach, 3),
                         flattened=len(self.flattened), clutter_zones=len(self.clutter['zones']),
                         pillars=self.clutter['pillars'], struts=len(self.clutter['struts']), lwalls=self.clutter['lwalls'],
                         explorable_m3=round(float(self.explorable.sum() * res ** 3), 1),
                         reachable_share=round(float(reach.sum() / cfree.sum()), 4))

    # ---------------------------------------------------------------------------------------------------------
    def _build(self, free, regions, solid_lab, islands, floor, ceil, gates, decks=()):
        """Occupancy [z, y, x], the floor and ceiling maps (m, absolute, NaN on full solid), and the start level's z."""
        from scipy import ndimage
        p, res = self.params, self.params.resolution
        base = -min(min(floor.values()), 0.0)
        top = max(ceil.values())
        ny, nx = free.shape
        fmap = np.full((ny, nx), np.nan)
        cmap = np.full((ny, nx), np.nan)
        for i in floor:
            m = regions == i
            fmap[m], cmap[m] = floor[i] + base, ceil[i] + base
        for j, isl in islands.items():
            mask = solid_lab == j
            fl = max(floor[i] for i in isl['around']) + base
            cl = min(ceil[i] for i in isl['around']) + base
            if isl['kind'] == 'low':
                hi = cl - self.gap
                lo = fl + p.low_island[1]
                if hi > lo:
                    fmap[mask] = math.floor((lo + isl['frac'] * (hi - lo)) / res) * res   # down: never less than the gap
                    cmap[mask] = cl
            elif isl['kind'] == 'lip':
                reach = max(1, int(round(isl['reach'] / res)))
                lip = ndimage.binary_dilation(mask, iterations=reach) & free
                lo, hi = fl + self.gap, cl - 1.0
                if hi > lo:
                    z = math.ceil((lo + isl['frac'] * (hi - lo)) / res) * res              # up: never less than the gap
                    cmap[lip] = np.minimum(cmap[lip], z)
                    fl_lip = fmap[lip]
                    bad = cmap[lip] - fl_lip < self.gap       # a lip over a raised floor: keep the region's own ceiling
                    if bad.any():
                        sub = cmap[lip]
                        sub[bad] = np.nan
                        cmap[lip] = np.where(np.isnan(sub), np.array([ceil[r] + base for r in regions[lip]]), sub)
        for g in gates:
            a_, b_ = g['pair']
            lo, hi = max(floor[a_], floor[b_]) + base, min(ceil[a_], ceil[b_]) + base
            m = g['strip']
            if g['kind'] == 'hurdle':
                fmap[m] = np.maximum(fmap[m], hi - self.gap)
            else:
                cmap[m] = np.minimum(cmap[m], lo + self.gap)
        nz = int(math.ceil((top + base) / res)) + 1
        zc = (np.arange(nz) + 0.5) * res
        f = np.where(np.isnan(fmap), np.inf, fmap)
        c = np.where(np.isnan(cmap), -np.inf, cmap)
        free3 = (zc[:, None, None] > f[None]) & (zc[:, None, None] < c[None])
        th = max(1, int(round(p.strut_thick / res)))
        for st in self.clutter['struts']:                 # struts: a beam over free columns, passage above and below
            ys, xs = np.nonzero(st['mask'] & ~np.isnan(fmap) & ~np.isnan(cmap))
            if not len(ys):
                continue
            fl, cl = fmap[ys, xs], cmap[ys, xs]
            lo, hi = fl + self.gap, cl - self.gap - th * res           # beam bottom range keeping both passages
            z = np.where(hi >= lo, lo + st['frac'] * (hi - lo), fl + self.gap)   # too low a room: passage below only
            k0 = np.ceil(z / res - 1e-6).astype(int)
            ok = (k0 + th) * res <= cl - 1e-6
            for dz in range(th):
                free3[np.clip(k0[ok] + dz, 0, nz - 1), ys[ok], xs[ok]] = False
        for d in decks:                                   # bridge: an enclosed passage over the block, raised ends
            rp, rq = int(regions[d['P']]), int(regions[d['Q']])
            z0 = max(floor[rp], floor[rq]) + base + d['rise']
            z1 = z0 + d['height']
            free3[(zc > z0) & (zc < z1)] |= d['upper_band'][None]
            f_end = np.where(np.isnan(fmap), np.inf, fmap)
            free3 |= d['ends'][None] & (zc[:, None, None] > f_end[None]) & (zc[:, None, None] < z1)
            d['z'] = (z0, z1)
        return ~free3, fmap, cmap, base

    def _start_point(self, rc, fmap, base):
        res = self.params.resolution
        f = fmap[rc]
        if np.isnan(f):
            f = base
        z = f + self.params.start_clearance
        z = (math.floor(z / res) + 0.5) * res
        return np.array([(rc[1] + 0.5) * res, (rc[0] + 0.5) * res, z])

    # ---------------------------------------------------------------------------------------------------------
    def save(self, folder):
        folder = Path(folder)
        folder.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(folder / 'world.npz', occupied=self.occupied, explorable=self.explorable, surface=self.surface,
                            start=self.start, origin=self.origin, resolution=self.params.resolution,
                            floor_map=self.floor_map, ceiling_map=self.ceil_map, regions=self.regions, base=self.base)
        (folder / 'stats.json').write_text(json.dumps(dict(self.info, params={k: (list(v) if isinstance(v, tuple) else v)
                                                                             for k, v in asdict(self.params).items()}), indent=1))

    def preview(self, path):
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        from mpl_toolkits.axes_grid1 import make_axes_locatable
        res, base = self.params.resolution, self.base
        ny, nx = self.free2d.shape
        ext = (0, nx * res, 0, ny * res)
        fig, axs = plt.subplots(1, 6, figsize=(29, 5.4), gridspec_kw=dict(width_ratios=[1, 1, 1, 1.1, 1.1, 0.9]))
        rng = np.random.default_rng(1)
        lut = rng.uniform(0.35, 0.95, (self.regions.max() + 1, 3))
        lut[0] = 0.25
        panels = [('regions (watershed)', lut[self.regions], None, None),
                  ('floor, m from the start level (islands: their top)', self.floor_map - base, 'RdBu_r', (-9, 9)),
                  ('ceiling, m from the start level', self.ceil_map - base, 'viridis', (-3, self.top))]
        for ax, (title, data, cmap, lim) in zip(axs, panels):
            if cmap is None:
                ax.imshow(data, origin='lower', extent=ext, interpolation='nearest')
            else:
                ax.set_facecolor('#3b3f47')
                im = ax.imshow(np.ma.masked_invalid(data), origin='lower', extent=ext, cmap=cmap, vmin=lim[0], vmax=lim[1],
                               interpolation='nearest')
                cax = make_axes_locatable(ax).append_axes('right', size='3%', pad=0.05)
                fig.colorbar(im, cax=cax).ax.tick_params(labelsize=7)
            for g in self.gates:
                ys, xs = np.nonzero(g['strip'])
                ax.scatter((xs + 0.5) * res, (ys + 0.5) * res, s=2, marker='s', linewidths=0,
                           color='#ff8c00' if g['kind'] == 'hurdle' else '#00c8ff')
            shafts = [r for r, k in self.kind.items() if k == 'shaft']
            for r in shafts:
                ax.contour((self.regions == r).astype(float), levels=[0.5], origin='lower', extent=ext,
                           colors='#ff2bd6', linewidths=1.6)
            for d in self.bridges:
                ax.contour(d['upper_band'].astype(float), levels=[0.5], origin='lower', extent=ext,
                           colors='white', linewidths=1.4, linestyles='--')
            ax.plot(self.start[0], self.start[1], marker='*', ms=15, color='#e0218a', mec='white', mew=0.8)
            ax.set_title(title, fontsize=10)
            ax.tick_params(labelsize=8)
        for st in self.clutter['struts']:                 # struts over the regions panel (yellow)
            ys, xs = np.nonzero(st['mask'])
            axs[0].scatter((xs + 0.5) * res, (ys + 0.5) * res, s=1.5, marker='s', linewidths=0, color='#ffd21f')
        # a side cut: through the first shaft (along its longer side), else through the start
        ax = axs[3]
        shafts = [r for r, k in self.kind.items() if k == 'shaft']
        if shafts:
            ys, xs = np.nonzero(self.regions == shafts[0])
            along_x = np.ptp(xs) >= np.ptp(ys)
            line = int(np.median(ys if along_x else xs))
            cut = self.occupied[:, line, :] if along_x else self.occupied[:, :, line]
            what = 'a shaft (magenta outline), %s = %.1f m' % ('y' if along_x else 'x', (line + 0.5) * res)
            span = (nx if along_x else ny) * res
            if along_x:
                axs[0].axhline((line + 0.5) * res, color='#ff2bd6', lw=0.8, ls='--')
            else:
                axs[0].axvline((line + 0.5) * res, color='#ff2bd6', lw=0.8, ls='--')
        else:
            line = int(self.start[1] // res)
            cut = self.occupied[:, line, :]
            what = 'the start, y = %.1f m' % self.start[1]
            span = nx * res
            ax.plot(self.start[0], self.start[2], marker='*', ms=15, color='#e0218a', mec='white', mew=0.8)
        ax.imshow(np.where(cut, 0.25, 0.97), origin='lower', cmap='gray', vmin=0, vmax=1,
                  extent=(0, span, 0, cut.shape[0] * res), interpolation='nearest', aspect='auto')
        ax.set_title('side cut through %s' % what, fontsize=10)
        ax.tick_params(labelsize=8)
        # plan of a bridge: lower route (the corridor, orange) and upper route (over the block, blue)
        ax = axs[4]
        if self.bridges:
            d = self.bridges[0]
            img = np.where(self.free2d, 0.95, 0.30)
            ax.imshow(img, origin='lower', extent=ext, cmap='gray', vmin=0, vmax=1, interpolation='nearest')
            for m_, cn, col, ls, lab in ((d['lower_band'], d['c_low'], '#e07800', '-', 'lower route: the corridor'),
                                         (d['upper_band'], d['c_up'], '#0d8fd0', '--', 'upper route: over the block')):
                ax.contourf(m_.astype(float), levels=[0.5, 1.5], origin='lower', extent=ext, colors=[col], alpha=0.25)
                pts = np.array([d['P'], cn, d['Q']], float)[:, ::-1] * res + res / 2
                ax.plot(pts[:, 0], pts[:, 1], color=col, lw=2.0, ls=ls, marker='o', ms=4, label=lab)
            ax.contour(d['ends'].astype(float), levels=[0.5], origin='lower', extent=ext, colors=['#ff2bd6'], linewidths=1.2)
            pts = np.array([d['P'], d['Q']], float)
            lo_, hi_ = pts.min(0) - 30, pts.max(0) + 30
            ax.set_xlim(max(0, lo_[1] * res), min(nx * res, hi_[1] * res))
            ax.set_ylim(max(0, lo_[0] * res), min(ny * res, hi_[0] * res))
            ax.legend(fontsize=7.5, loc='upper right', framealpha=0.9)
            ax.set_title('plan of a bridge: two different L routes between the same two ends\n(magenta: ends opened up to '
                         'the bridge)', fontsize=10)
        else:
            ax.axis('off')
            ax.set_title('no bridge in this world', fontsize=10)
        ax.tick_params(labelsize=8)
        # side cut along the bridge's first leg
        ax = axs[5]
        if self.bridges:
            d = self.bridges[0]
            (pr, pc), (cr, cc) = d['P'], d['c_up']
            if pr == cr:                                  # first leg runs along x at row pr
                cut, span, lo_, hi_, lab = self.occupied[:, pr, :], nx * res, min(pc, cc), max(pc, cc), 'y = %.1f m' % ((pr + .5) * res)
            else:                                         # first leg runs along y at column pc
                cut, span, lo_, hi_, lab = self.occupied[:, :, pc], ny * res, min(pr, cr), max(pr, cr), 'x = %.1f m' % ((pc + .5) * res)
            ax.imshow(np.where(cut, 0.25, 0.97), origin='lower', cmap='gray', vmin=0, vmax=1,
                      extent=(0, span, 0, cut.shape[0] * res), interpolation='nearest', aspect='auto')
            ax.set_xlim(max(0, (lo_ - 15) * res), min(span, (hi_ + 15) * res))
            ax.axhspan(*d['z'], color='#0d8fd0', alpha=0.12)
            ax.set_title('side cut along the upper route\'s first leg (%s):\nthe bridge passage (blue band) through the '
                         'block' % lab, fontsize=10)
        else:
            ax.axis('off')
        ax.tick_params(labelsize=8)
        i = self.info
        fig.suptitle('%s (seed %d): %.0f x %.0f m, %.1f m tall; %d regions on %d levels (span %.1f m); %d staggers (shafts, '
                     'magenta); %d bridges (white); %d pits; %d islands (%d low, %d lips); %d hurdles (orange), '
                     '%d lintels (cyan); best single height reaches %.0f%%\nclutter: %d zones, %d pillars, %d strut layers (yellow), %d L walls'
                     % (i['png'], self.seed, *i['size_m'], i['regions'], i['levels'], i['level_span_m'], i['staggers'],
                        i['bridges'], i['sunken'], i['islands'], i['low_islands'], i['lips'], i['hurdles'],
                        i['lintels'], 100 * i['flat_reach'], i['clutter_zones'], i['pillars'], i['struts'], i['lwalls']),
                     fontsize=10.5)
        fig.tight_layout(rect=(0, 0, 1, 0.93))
        fig.savefig(path, dpi=80)
        plt.close(fig)


# ---- helpers -------------------------------------------------------------------------------------------------

def _largest_containing(free, rc, ndimage):
    lab, _ = ndimage.label(free)
    if lab[rc] == 0:
        raise ValueError('start is not in free space')
    return lab == lab[rc]


def _merge_small(regions, min_cells):
    """Join regions under min_cells to the neighbour they touch most (repeated until none is left)."""
    while True:
        ids, counts = np.unique(regions[regions > 0], return_counts=True)
        small = ids[counts < min_cells]
        if len(small) == 0 or len(ids) <= 1:
            return regions
        i = small[np.argmin(counts[counts < min_cells])]
        m = regions == i
        ring = np.zeros_like(m)
        ring[1:] |= m[:-1]; ring[:-1] |= m[1:]; ring[:, 1:] |= m[:, :-1]; ring[:, :-1] |= m[:, 1:]
        nb = regions[ring & ~m]
        nb = nb[nb > 0]
        if len(nb) == 0:
            regions[m] = 0 if len(ids) > 1 else i
            continue
        regions[m] = np.bincount(nb).argmax()


def _plan_bridges(free, regions, solid_lab, islands, rng, p, res, snap, gap, ndimage):
    """Up to max_bridges bridges. A bridge joins free points P and Q offset bridge_span in both x and y. The L from P
    to Q turning at one corner of their rectangle runs along free space (the corridor: the lower route); the L turning
    at the other corner runs mostly through solid (the block the corridor wraps around: the upper route, carved as an
    enclosed passage above the corridor floors). Around P and Q the space is opened up to the bridge."""
    ny, nx = free.shape
    yy, xx = np.indices(free.shape)
    dist = ndimage.distance_transform_edt(free)
    lo_c, hi_c = (int(round(v / res)) for v in p.bridge_span)
    out, used = [], np.zeros_like(free)

    def band(a, b, hw):
        return (yy >= min(a[0], b[0]) - hw) & (yy <= max(a[0], b[0]) + hw) & \
               (xx >= min(a[1], b[1]) - hw) & (xx <= max(a[1], b[1]) + hw)

    for _ in range(p.bridge_tries):
        if len(out) >= p.max_bridges:
            break
        w = max(gap, snap(rng.uniform(*p.bridge_width)))
        hw = max(1, int(round(w / res / 2)))
        centre = np.argwhere((dist >= hw + 1) & ~used)
        if len(centre) == 0:
            break
        P = tuple(centre[rng.integers(len(centre))])
        dr, dc = rng.integers(lo_c, hi_c + 1) * rng.choice([-1, 1]), rng.integers(lo_c, hi_c + 1) * rng.choice([-1, 1])
        Q = (P[0] + int(dr), P[1] + int(dc))
        if not (0 <= Q[0] < ny and 0 <= Q[1] < nx) or dist[Q] < hw + 1 or used[Q]:
            continue
        for c_low, c_up in (((P[0], Q[1]), (Q[0], P[1])), ((Q[0], P[1]), (P[0], Q[1]))):
            low = band(P, c_low, hw) | band(c_low, Q, hw)
            up = band(P, c_up, hw) | band(c_up, Q, hw)
            if free[low].mean() < 0.9 or not free[c_low] or free[c_up]:
                continue                                  # the lower L must be corridor, its corner free; the other solid
            r_end = max(hw + 2, int(math.ceil(3.0 / res)))
            ends = free & (((np.abs(yy - P[0]) <= r_end) & (np.abs(xx - P[1]) <= r_end)) |   # square end zones
                           ((np.abs(yy - Q[0]) <= r_end) & (np.abs(xx - Q[1]) <= r_end)))
            mid = up & ~ends & ~ndimage.binary_dilation(ends, iterations=hw)
            if mid.sum() == 0 or (~free[mid]).mean() < 0.8:
                continue                                  # the upper L must cross the block, not other open space
            lab_up = ndimage.label(up | ends)[0]
            if lab_up[P] == 0 or lab_up[P] != lab_up[Q]:
                continue
            for j in np.unique(solid_lab[up & ~free]):    # blocks under the bridge stay full height
                if j in islands:
                    islands[j]['kind'] = 'full'
            out.append(dict(P=P, Q=Q, c_low=c_low, c_up=c_up, width=w, lower_band=low & free, upper_band=up,
                            ends=ends, rise=snap(rng.uniform(*p.bridge_rise)),
                            height=max(gap, snap(rng.uniform(*p.bridge_height)))))
            used |= ndimage.binary_dilation(up | low, iterations=int(round(6 / res)))
            break
    return out


def _joined_by_shaft(a, b, adj, kind, floor, ceil, gap):
    """True when touching regions a and b share less than `gap` of height but a shaft next to both spans them."""
    if kind[a] == 'shaft' or kind[b] == 'shaft':
        return False
    if min(ceil[a], ceil[b]) - max(floor[a], floor[b]) >= gap:
        return False
    for s in adj[a] & adj[b]:
        if kind[s] == 'shaft' and floor[s] <= min(floor[a], floor[b]) + 1e-6 and \
                ceil[s] >= max(floor[a], floor[b]) + gap - 1e-6:
            return True
    return False


def _boundary(regions, a, b):
    """Cells of region a next to region b and of b next to a (4-neighbour)."""
    ma, mb = regions == a, regions == b
    def grow(m):
        g = m.copy()
        g[1:] |= m[:-1]; g[:-1] |= m[1:]; g[:, 1:] |= m[:, :-1]; g[:, :-1] |= m[:, 1:]
        return g
    return (ma & grow(mb)) | (mb & grow(ma))


def _flat_reach(cfree):
    """Largest share of the footprint (columns with any free configuration) one constant height reaches in one
    4-connected piece: (share, layer index, that layer's labels)."""
    from scipy import ndimage
    foot = cfree.any(0).sum()
    best = (0.0, 0, None)
    for k in range(cfree.shape[0]):
        if not cfree[k].any():
            continue
        lab, _ = ndimage.label(cfree[k])
        share = np.bincount(lab.ravel())[1:].max() / max(foot, 1)
        if share > best[0]:
            best = (float(share), k, lab)
    return best


def _touching_pairs(regions):
    pairs = set()
    for a, b in ((regions[:, :-1], regions[:, 1:]), (regions[:-1, :], regions[1:, :])):
        m = (a != b) & (a > 0) & (b > 0)
        pairs |= {tuple(sorted(t)) for t in zip(a[m].tolist(), b[m].tolist())}
    return sorted(pairs)


def _cspace(occ, robot, res, ndimage):
    """Cells where the drone's box fits: free cells at least robot/2 from any occupied cell centre (box metric)."""
    r = max(1, int(math.ceil((robot / 2) / res - 0.5)))
    grown = ndimage.binary_dilation(occ, structure=np.ones((2 * r + 1,) * 3, bool))
    return ~grown


def _largest_free_from(free, si, ndimage):
    lab, _ = ndimage.label(free)
    return lab == lab[si]


def _dilate6(mask):
    out = mask.copy()
    out[1:] |= mask[:-1]
    out[:-1] |= mask[1:]
    out[:, 1:] |= mask[:, :-1]
    out[:, :-1] |= mask[:, 1:]
    out[:, :, 1:] |= mask[:, :, :-1]
    out[:, :, :-1] |= mask[:, :, 1:]
    return out


def _disk(r):
    yy, xx = np.mgrid[-r:r + 1, -r:r + 1]
    return yy * yy + xx * xx <= r * r


def _plan_clutter(free, dist, start_rc, p, rng, ndimage):
    """Pillars (+ struts) and L walls in the big open zones of `free` (see Ari3DParams.clutter). Returns dict(mask: the
    new solid cells, hall_mask: the pillar zones, struts: [dict(mask, frac)] one per strut layer, zones: [dict(kind,
    area_m2, ...)], pillars, lwalls, strut_links)."""
    res = p.resolution
    ny, nx = free.shape
    r_open = p.open_radius / res
    core = free & (dist >= r_open)
    zone = free & (ndimage.distance_transform_edt(~core) <= r_open) if core.any() else np.zeros_like(free)
    lab, n = ndimage.label(zone)
    g = int(round(p.clutter_gap / res))                   # free cells to keep between an L wall and other solid
    gp = int(round(p.pillar_gap / res))                   # ... and between a pillar and other solid
    er = _disk(max(1, g // 2))
    cur = free.copy()
    mask = np.zeros_like(free)
    hall = np.zeros_like(free)
    sy0, sx0 = start_rc
    yy, xx = np.mgrid[0:ny, 0:nx]
    near_start = (yy - sy0) ** 2 + (xx - sx0) ** 2 <= (3.0 / res) ** 2
    pieces = lambda fr: ndimage.label(ndimage.binary_erosion(fr, er))[1]
    n_pieces = [pieces(cur)]
    out = dict(mask=mask, hall_mask=hall, struts=[], zones=[], pillars=0, lwalls=0, strut_links=0)

    def stamp(S, exempt=None, check_pieces=False, g=g):
        if not S.any() or (S & ~cur).any() or (S & near_start).any():
            return False
        ys, xs = np.nonzero(S)
        y0, y1, x0, x1 = max(0, ys.min() - g), min(ny, ys.max() + g + 1), max(0, xs.min() - g), min(nx, xs.max() + g + 1)
        core_s = S[y0:y1, x0:x1] & ~(exempt[y0:y1, x0:x1] if exempt is not None else False)
        grown = ndimage.binary_dilation(core_s, _disk(g))
        if (grown & ~cur[y0:y1, x0:x1] & ~S[y0:y1, x0:x1]).any():
            return False                                   # closer than clutter_gap to something solid
        if check_pieces:
            k = pieces(cur & ~S)
            if k != n_pieces[0]:
                return False                               # cut off a pocket (or closed a gap) for the grown drone
        cur[S] = False
        mask[S] = True
        return True

    dirs = [(0, 1), (0, -1), (1, 0), (-1, 0)]
    order = sorted(range(1, n + 1), key=lambda z: -(lab == z).sum())
    for z in order:
        zm = lab == z
        area = float(zm.sum()) * res * res
        if area < p.clutter_min_m2 or rng.random() > p.clutter_chance:
            continue
        wts = np.asarray(p.clutter_weights if len(p.clutter_weights) == len(p.clutter) else [1.0] * len(p.clutter), float)
        kind = p.clutter[int(rng.choice(len(p.clutter), p=wts / wts.sum()))]
        info = dict(kind=kind, area_m2=round(area, 1))
        made, placed = 0, {}
        if kind in ('lwalls', 'mixed'):
            n_l = int(np.clip(round(area / p.lwall_area), 1, p.lwall_max))
            wall_adj = zm & (dist <= 1.5)
            inner = zm & (ndimage.distance_transform_edt(cur) >= g + 1)
            for _ in range(n_l):
                for _try in range(60):
                    attach = rng.random() < p.lwall_attach
                    t = max(1, int(round(rng.uniform(*p.lwall_thick) / res)))
                    L1, L2 = (int(round(rng.uniform(*p.lwall_leg) / res)) for _ in range(2))
                    pool = np.argwhere(wall_adj & cur if attach else inner & cur)
                    if not len(pool):
                        continue
                    c = pool[int(rng.integers(len(pool)))]
                    if attach:                             # first leg points away from the wall it starts at
                        opts = [d for d in dirs if not (0 <= c[0] - d[0] < ny and 0 <= c[1] - d[1] < nx)
                                or not free[c[0] - d[0], c[1] - d[1]]]
                        if not opts:
                            continue
                        d1 = opts[int(rng.integers(len(opts)))]
                    else:
                        d1 = dirs[int(rng.integers(4))]
                    d2s = [d for d in dirs if d[0] * d1[0] + d[1] * d1[1] == 0]
                    d2 = d2s[int(rng.integers(2))]
                    S = np.zeros_like(free)
                    ex = np.zeros_like(free)
                    cells = [(c[0] + k * d1[0] + m * d2[0], c[1] + k * d1[1] + m * d2[1], k) for k in range(L1 + 1)
                             for m in range(t)]
                    cells += [(c[0] + (L1 - m) * d1[0] + k * d2[0], c[1] + (L1 - m) * d1[1] + k * d2[1], L1)
                              for k in range(L2 + 1) for m in range(t)]
                    if any(not (0 <= y < ny and 0 <= x < nx) for y, x, _ in cells):
                        continue
                    for y, x, k in cells:
                        S[y, x] = True
                        if attach and k < g:
                            ex[y, x] = True
                    if stamp(S, ex if attach else None, check_pieces=True):
                        made += 1
                        break
            out['lwalls'] += made
            info.update(lwalls=made)
        if kind in ('pillars', 'mixed'):
            skip = p.mixed_pillar_skip if kind == 'mixed' else p.pillar_skip
            ys, xs = np.nonzero(zm)
            px, py = rng.uniform(*p.pillar_spacing) / res, rng.uniform(*p.pillar_spacing) / res
            ox, oy = rng.uniform(0, px), rng.uniform(0, py)
            jit = p.pillar_jitter / res
            placed = {}
            for i, gy in enumerate(np.arange(ys.min() + oy, ys.max() + 1, py)):
                for j, gx in enumerate(np.arange(xs.min() + ox, xs.max() + 1, px)):
                    if rng.random() < skip:
                        continue
                    cy, cx = int(round(gy + rng.uniform(-jit, jit))), int(round(gx + rng.uniform(-jit, jit)))
                    if not (0 <= cy < ny and 0 <= cx < nx and zm[cy, cx]):
                        continue
                    s = max(1, int(round(rng.uniform(*p.pillar_size) / res)))
                    S = np.zeros_like(free)
                    S[max(0, cy - s // 2):cy - s // 2 + s, max(0, cx - s // 2):cx - s // 2 + s] = True
                    if stamp(S, g=gp):
                        placed[(i, j)] = (cy, cx)
        if placed:
            hall |= zm
            out['pillars'] += len(placed)
            th = max(1, int(round(p.strut_thick / res)))
            n_layers = int(rng.integers(p.strut_layers[0], p.strut_layers[1] + 1))
            fracs = [(k + rng.uniform(0.2, 0.8)) / n_layers for k in range(n_layers)]
            links = 0
            for frac in fracs:
                sm = np.zeros_like(free)

                def beam(a, b):
                    m = int(max(abs(b[0] - a[0]), abs(b[1] - a[1]))) + 1
                    for t in np.linspace(0, 1, m):
                        y, x = int(round(a[0] + t * (b[0] - a[0]))), int(round(a[1] + t * (b[1] - a[1])))
                        sm[max(0, y - th // 2):y - th // 2 + th, max(0, x - th // 2):x - th // 2 + th] = True

                for (i, j), a in placed.items():
                    for di, dj in ((1, 0), (0, 1), (-1, 0), (0, -1)):
                        b = placed.get((i + di, j + dj))
                        if b is not None:
                            if (di, dj) in ((1, 0), (0, 1)) and rng.random() < p.strut_chance:
                                beam(a, b)
                                links += 1
                            continue
                        # no pillar that way: a strut to the wall, if one is within 1.5 pitches
                        step = (di, dj)
                        reach = int(1.5 * (py if di else px))
                        for k in range(1, reach + 1):
                            y, x = a[0] + k * step[0], a[1] + k * step[1]
                            if not (0 <= y < ny and 0 <= x < nx):
                                break
                            if not free[y, x] and not mask[y, x]:  # a wall of the map (not clutter)
                                if rng.random() < p.strut_chance:
                                    beam(a, (y - step[0], x - step[1]))
                                    links += 1
                                break
                sm &= cur
                if sm.any():
                    out['struts'].append(dict(mask=sm, frac=float(frac)))
            out['strut_links'] += links
            info.update(pillars=len(placed), strut_layers=n_layers, strut_links=links)
        if not made and not placed:
            continue
        out['zones'].append(info)
    return out
