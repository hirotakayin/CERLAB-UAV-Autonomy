"""Procedural 3D obstacle courses for training and evaluating exploration policies.

A course is a closed room (floor, four walls, ceiling) with obstacles at every height. Most are anchored to the
room: pillars on a loose grid (floor to ceiling), blocks standing on the floor, hanging from the ceiling or sticking
out of a wall, beams from wall to wall or cantilevered from one, ledges sticking out of a wall, and at most one
dividing wall with large openings. Only a few blocks float free. Obstacles keep min_gap apart, so the room stays
open enough to explore. Everything is a box, so a course plugs straight
into BoxScene (the exact ray model checked against Gazebo's Mid-360) and exports to an SDF model Gazebo can load.

Courses are deterministic in (seed, CourseParams). Every obstacle is accepted only if the drone can still reach
almost all of the room from the start: reachability is a flood fill over configuration space, i.e. free cells
where the drone's safety box (robot_size, 1 m) fits. The ground truth kept with each course is

  explorable  free cells connected to the start through free space (6-neighbour flood fill on the fine grid);
              sealed pockets are left out, so 100% coverage is achievable in principle
  surface     occupied cells next to an explorable cell: the surfaces there are to see

Seed ranges split the data: test 0-99, validation 100-199, training 1000 and up.
"""

from dataclasses import asdict, dataclass, field
import json
import math
from pathlib import Path
from xml.sax.saxutils import escape

import numpy as np

from .world import BoxCollision

SPLITS = {'test': range(0, 100), 'val': range(100, 200), 'train': range(1000, 10 ** 9)}
KINDS = ('room', 'pillar', 'block', 'beam', 'ledge', 'floating', 'wall', 'maze', 'step', 'soffit',
         'hurdle', 'lintel', 'bar', 'shelf')
STRUCTURE = ('room', 'maze', 'step', 'soffit')   # the layout itself, not obstacles


@dataclass
class CourseParams:
    size_xy: tuple = (20.0, 30.0)       # m, room length and width are each drawn from this range
    height: tuple = (6.0, 10.0)         # m, floor to ceiling
    start: str = 'floor'                # 'floor' (start_clearance above the floor) or 'ceiling' (below the ceiling)
    start_clearance: float = 0.6        # m from the floor or ceiling to the drone's centre (0.5 = touching)
    difficulty: float = 0.5             # 0..1, scales the obstacle counts
    count_jitter: int = 1               # each count varies by up to this much per course
    min_gap: float = 2.0                # m, clearance between any two obstacles (the room shell excluded)
    pillar_spacing: tuple = (5.0, 8.0)  # m, the pillar grid's pitch; pillars take some of its points
    yaw_blocks: bool = True             # blocks standing on the floor and floating blocks get a random yaw
    robot_size: float = 1.0             # m, the drone's safety box (mapping_param robot_size)
    wall_thickness: float = 0.4         # m, room shell
    check_resolution: float = 0.4       # m, configuration-space grid for the reachability check
    resolution: float = 0.2             # m, ground-truth grid (the scoring map's resolution)
    min_reachable: float = 0.95         # share of the free configuration space that must stay reachable from the start
    counts: dict = field(default_factory=lambda: {   # (count at difficulty 0, count at difficulty 1)
        'pillar': (2, 6), 'block': (2, 5), 'beam': (1, 3), 'ledge': (1, 4), 'floating': (0, 2), 'wall': (0, 1)})
    # layout 'maze': winding corridors and rooms on a coarse cell grid instead of one open room (see _maze)
    layout: str = 'room'                # 'room' or 'maze'
    maze_cell: tuple = (6.0, 8.0)       # m, target cell pitch; the room is split into whole cells
    corridor_width: tuple = (3.0, 4.5)  # m, per corridor segment
    room_share: float = 0.4             # share of cells carved out whole as chambers
    loop_share: float = 0.15            # share of the spanning tree's left-out neighbour pairs opened as extra links
    inner_wall: float = 0.4             # m, wall between neighbouring room cells
    # heights are above the corridor floor; sunken chambers go below it, so the course's z = 0 is the deepest floor
    low_ceiling: tuple = (0.4, 3.5)     # (share of cells, lowest ceiling in m): cells with a ceiling below the roof
    sunken: tuple = (0.3, 3.0, 6.0)     # (share of chambers, shallowest, deepest drop in m)
    raised: tuple = (0.25, 1.5, 3.0)    # (share of chambers, lowest, highest raise in m)
    min_overlap: float = 2.5            # m, the height two linked cells share at least (so the drone can pass)
    maze_counts: dict = field(default_factory=lambda: {   # large obstacles, each forcing a height choice
        'hurdle': (1, 3), 'lintel': (1, 3), 'bar': (0, 2), 'shelf': (1, 3)})


@dataclass
class Box:
    kind: str
    center: np.ndarray
    size: np.ndarray
    yaw: float = 0.0
    group: int = -1          # pieces of one partial wall share a group

    def collision(self, i):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        rot = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
        return BoxCollision('%s_%d' % (self.kind, i), np.asarray(self.size, float), rot, np.asarray(self.center, float))


class Course:
    """A generated room: boxes, the start, and the ground-truth grids (see the module docstring)."""

    def __init__(self, seed, params, room, boxes, start, rejected, info=None):
        self.seed, self.params, self.room, self.boxes, self.start, self.rejected = seed, params, room, boxes, start, rejected
        self.info = info or {}   # layout facts (maze: grid, rooms, links, height maps)
        # the room's inner corner sits at the world origin (interior [0, room]); the ground-truth grid reaches one
        # cell past it on every side, so the floor, walls and ceiling are in it as occupied cells
        self.origin = -np.full(3, params.resolution)
        self.extent = room + 2 * params.resolution
        self.occupied, self.explorable = self._ground_truth()
        self.surface = self.occupied & _dilate6(self.explorable)

    @property
    def collisions(self):
        return [b.collision(i) for i, b in enumerate(self.boxes)]

    def scene(self):
        from .fast import BoxScene
        return BoxScene(self.collisions)

    def _ground_truth(self):
        r = self.params.resolution
        occ = _rasterize(self.boxes, self.origin, self.extent, r, inflate=0.0)
        start_idx = tuple(np.floor((self.start - self.origin) / r).astype(int)[::-1])  # grids are [z, y, x]
        return occ, _flood(~occ, start_idx)

    def stats(self):
        r3 = self.params.resolution ** 3
        kinds = {k: sum(b.kind == k for b in self.boxes) for k in KINDS if k not in STRUCTURE + ('wall',)}
        if self.params.layout == 'room':
            kinds['wall'] = len({b.group for b in self.boxes if b.kind == 'wall'})
        else:
            kinds = {k: v for k, v in kinds.items() if k in self.params.maze_counts}
        layout = {k: v for k, v in self.info.items() if not isinstance(v, np.ndarray)}
        return dict(seed=self.seed, layout=self.params.layout, room_m=[round(float(v), 2) for v in self.room],
                    start=[round(float(v), 2) for v in self.start], maze=layout or None,
                    obstacles=kinds, rejected_obstacles=self.rejected,
                    free_m3=round(float((~self.occupied).sum() * r3), 1),
                    explorable_m3=round(float(self.explorable.sum() * r3), 1),
                    explorable_share=round(float(self.explorable.sum() / max(1, (~self.occupied).sum())), 4),
                    surface_cells=int(self.surface.sum()), params=_jsonable(asdict(self.params)))

    def save(self, folder):
        """course.npz (boxes, start, grids), stats.json, and the Gazebo model: model.sdf, model.config, world.sdf."""
        folder = Path(folder)
        folder.mkdir(parents=True, exist_ok=True)
        name = 'course_%d' % self.seed
        extra = {k: v for k, v in self.info.items() if isinstance(v, np.ndarray)}
        np.savez_compressed(folder / 'course.npz', seed=self.seed, room=self.room, start=self.start, **extra,
                            kind=np.array([b.kind for b in self.boxes]), center=np.array([b.center for b in self.boxes]),
                            size=np.array([b.size for b in self.boxes]), yaw=np.array([b.yaw for b in self.boxes]),
                            origin=self.origin, extent=self.extent, resolution=self.params.resolution,
                            occupied=self.occupied, explorable=self.explorable, surface=self.surface)
        (folder / 'stats.json').write_text(json.dumps(self.stats(), indent=1))
        (folder / 'model.sdf').write_text(self.sdf(name))
        (folder / 'model.config').write_text(
            '<?xml version="1.0"?>\n<model>\n  <name>%s</name>\n  <version>1.0</version>\n  <sdf version="1.6">model.sdf</sdf>\n'
            '  <description>voxel_world obstacle course (%s), seed %d</description>\n</model>\n' % (name, self.params.layout, self.seed))
        (folder / 'world.sdf').write_text(self.world_sdf(name))

    def sdf(self, name):
        """One static model, one link per box (collision + visual), in the subset load_sdf_boxes reads."""
        colors = {'room': '0.75 0.75 0.75 1', 'pillar': '0.55 0.45 0.35 1', 'block': '0.30 0.45 0.70 1',
                  'beam': '0.70 0.35 0.25 1', 'ledge': '0.35 0.60 0.40 1', 'floating': '0.55 0.35 0.65 1',
                  'wall': '0.60 0.60 0.50 1', 'maze': '0.70 0.70 0.68 1', 'step': '0.62 0.55 0.45 1',
                  'soffit': '0.55 0.65 0.75 1', 'hurdle': '0.80 0.55 0.20 1', 'lintel': '0.35 0.55 0.80 1',
                  'bar': '0.75 0.30 0.30 1', 'shelf': '0.35 0.65 0.45 1'}
        links = []
        for i, b in enumerate(self.boxes):
            size = '%.4f %.4f %.4f' % tuple(b.size)
            links.append(
                '    <link name="%s_%d">\n      <pose>%.4f %.4f %.4f 0 0 %.6f</pose>\n'
                '      <collision name="collision"><geometry><box><size>%s</size></box></geometry></collision>\n'
                '      <visual name="visual"><geometry><box><size>%s</size></box></geometry>'
                '<material><ambient>%s</ambient><diffuse>%s</diffuse></material></visual>\n    </link>'
                % (b.kind, i, *b.center, b.yaw, size, size, colors[b.kind], colors[b.kind]))
        return ('<?xml version="1.0"?>\n<sdf version="1.6">\n  <model name="%s">\n    <static>true</static>\n%s\n  </model>\n</sdf>\n'
                % (escape(name), '\n'.join(links)))

    def world_sdf(self, name):
        return ('<?xml version="1.0"?>\n<sdf version="1.6">\n  <world name="%s">\n'
                '    <include><uri>model://sun</uri></include>\n'
                '    <include><uri>model://%s</uri></include>\n  </world>\n</sdf>\n' % (escape(name), escape(name)))


def generate(seed, params=None):
    """Build the course for one seed. Obstacles that would cut the drone off from more than (1 - min_reachable) of
    the room are drawn again (up to a limit) and then skipped; the count of skipped draws is kept in stats."""
    p = params or CourseParams()
    if p.layout not in ('room', 'maze'):
        raise ValueError("layout must be 'room' or 'maze'")
    rng = np.random.default_rng(seed)
    room = np.array([rng.uniform(*p.size_xy), rng.uniform(*p.size_xy), rng.uniform(*p.height)])
    info, maze = {}, None
    if p.layout == 'maze':
        maze = _maze(rng, room, p)       # may deepen room[2] for sunken chambers
        info = maze.info
    boxes = _shell(room, p.wall_thickness) + (maze.boxes if maze else [])
    cr = p.check_resolution
    shape = np.ceil(room / cr).astype(int)[::-1]
    cfree = ~_rasterize(boxes, np.zeros(3), room, cr, inflate=p.robot_size / 2)
    start = _pick_start(rng, p, room, cfree) if maze is None else maze.pick_start(rng, p, cfree)
    start_idx = tuple(np.floor(start / cr).astype(int)[::-1])

    state = dict(pillar_grid=_pillar_grid(rng, room, p)) if maze is None else dict(maze=maze)
    plan = []
    d = float(np.clip(p.difficulty, 0, 1))
    for kind, (lo, hi) in (p.counts if maze is None else p.maze_counts).items():
        n = int(math.floor(lo + (hi - lo) * d + 0.5)) + int(rng.integers(-p.count_jitter, p.count_jitter + 1))
        plan += [kind] * max(0, min(n, hi))
    # big pieces first, so the small ones fit around them
    order = {'wall': 0, 'pillar': 1, 'beam': 2, 'ledge': 3, 'block': 4, 'floating': 5,
             'shelf': 0, 'hurdle': 1, 'lintel': 1, 'bar': 2}
    plan.sort(key=lambda k: (order[k], rng.random()))
    rejected = 0
    for kind in plan:
        for _ in range(30):
            new = _sample(kind, rng, room, p, state) if maze is None else maze.sample(kind, rng, p)
            if new is None or not _clear_of(new, [b for b in boxes if b.kind not in STRUCTURE], p.min_gap):
                rejected += 1
                continue
            blocked = _rasterize(new, np.zeros(3), room, cr, inflate=p.robot_size / 2)
            trial = cfree & ~blocked
            if not trial[start_idx]:
                rejected += 1
                continue
            if _flood(trial, start_idx).sum() >= p.min_reachable * trial.sum():  # sealed-off pockets, not the obstacle itself
                boxes += new
                cfree = trial
                if kind == 'pillar' and maze is None:
                    state['pillar_grid'].pop(state.pop('pillar_pick'))
                break
            rejected += 1
    assert shape.tolist() == list(cfree.shape)
    return Course(seed, p, room, boxes, start, rejected, info)


# ---- maze layout ---------------------------------------------------------------------------------------------

class _Maze:
    """Corridors and rooms carved out of solid, on an nx x ny grid of cells.

    The cells are linked by a random spanning tree (depth-first, so the corridors wind and end in dead ends), plus
    a few extra links that make loops. A cell is either a room (carved whole, less half an inner wall on each side)
    or a corridor junction (a square as wide as its widest link); each link is a straight corridor between two cell
    centres. Every cell has its own floor (sometimes raised) and ceiling (sometimes below the roof), so the carved
    space is a height field: floor(x, y) and ceiling(x, y). The solid parts become boxes: full-height walls
    ('maze'), raised floors ('step') and lowered ceilings ('soffit'), merged into as few rectangles as possible."""

    PIX = 0.1   # m, the raster used to check obstacle footprints

    def __init__(self, rng, room, p):
        L, W, top = room
        self.room = room
        nx = max(2, int(round(L / rng.uniform(*p.maze_cell))))
        ny = max(2, int(round(W / rng.uniform(*p.maze_cell))))
        cx, cy = L / nx, W / ny
        self.nx, self.ny, self.cx, self.cy = nx, ny, cx, cy
        cells = [(i, j) for i in range(nx) for j in range(ny)]

        # spanning tree by randomised depth-first search, then extra links for loops
        nbrs = lambda c: [(c[0] + di, c[1] + dj) for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1))
                          if 0 <= c[0] + di < nx and 0 <= c[1] + dj < ny]
        first = cells[int(rng.integers(len(cells)))]
        seen, stack, links = {first}, [first], set()
        while stack:
            c = stack[-1]
            todo = [n for n in nbrs(c) if n not in seen]
            if not todo:
                stack.pop()
                continue
            n = todo[int(rng.integers(len(todo)))]
            links.add(tuple(sorted((c, n))))
            seen.add(n)
            stack.append(n)
        tree = len(links)
        rest = sorted({tuple(sorted((c, n))) for c in cells for n in nbrs(c)} - links)
        for pair in rest:
            if rng.random() < p.loop_share:
                links.add(pair)
        self.links = sorted(links)
        degree = {c: 0 for c in cells}
        for a, b in self.links:
            degree[a] += 1
            degree[b] += 1

        # per-cell profile (floor, ceiling) above the corridor floor, then the carved rectangles
        rooms = {c for c in cells if rng.random() < p.room_share}
        floor, ceil, kind_of = {}, {}, {}
        for c in cells:
            u = rng.random()
            if c in rooms and u < p.sunken[0]:
                kind_of[c], floor[c] = 'sunken', -rng.uniform(*p.sunken[1:])
            elif c in rooms and u < p.sunken[0] + p.raised[0]:
                kind_of[c], floor[c] = 'raised', rng.uniform(*p.raised[1:])
            else:
                kind_of[c], floor[c] = ('chamber' if c in rooms else 'corridor'), 0.0
            low = kind_of[c] != 'sunken' and rng.random() < p.low_ceiling[0]
            ceil[c] = top if not low else rng.uniform(max(p.low_ceiling[1], floor[c] + 3.0), max(p.low_ceiling[1] + 0.5, top - 1.5))
        for _ in range(3):                      # linked cells must share min_overlap of height: lift the lower ceiling
            for a_, b_ in self.links:
                need = max(floor[a_], floor[b_]) + p.min_overlap
                for c in (a_, b_):
                    if ceil[c] < need:
                        ceil[c] = min(top, need)
        base = -min(floor.values())             # shift so the deepest floor is z = 0
        floor = {c: v + base for c, v in floor.items()}
        ceil = {c: v + base for c, v in ceil.items()}
        H = top + base
        room[2] = H
        self.base, self.kind_of = base, kind_of
        width = {pair: rng.uniform(*p.corridor_width) for pair in self.links}
        centre = lambda c: np.array([(c[0] + 0.5) * cx, (c[1] + 0.5) * cy])
        carve = []                                       # (x0, x1, y0, y1)
        t = p.inner_wall / 2
        for c in cells:
            if c in rooms:
                carve.append((c[0] * cx + t, (c[0] + 1) * cx - t, c[1] * cy + t, (c[1] + 1) * cy - t))
            else:
                w = max([width[pr] for pr in self.links if c in pr] or [p.corridor_width[0]])
                m = centre(c)
                carve.append((m[0] - w / 2, m[0] + w / 2, m[1] - w / 2, m[1] + w / 2))
        for (a, b) in self.links:
            ma, mb, w = centre(a), centre(b), width[(a, b)]
            lo, hi = np.minimum(ma, mb) - w / 2, np.maximum(ma, mb) + w / 2
            if a[0] != b[0]:
                lo[0], hi[0] = ma[0], mb[0]
            else:
                lo[1], hi[1] = ma[1], mb[1]
            carve.append((lo[0], hi[0], lo[1], hi[1]))
        carve = [(max(0.0, x0), min(L, x1), max(0.0, y0), min(W, y1)) for x0, x1, y0, y1 in carve]

        # tiles between all rectangle edges and cell lines; each is solid or takes its cell's (floor, ceiling)
        xs = np.unique(np.round([0.0, L, *[cx * i for i in range(nx)], *[r[0] for r in carve], *[r[1] for r in carve]], 6))
        ys = np.unique(np.round([0.0, W, *[cy * j for j in range(ny)], *[r[2] for r in carve], *[r[3] for r in carve]], 6))
        xm, ym = (xs[:-1] + xs[1:]) / 2, (ys[:-1] + ys[1:]) / 2
        key = np.empty((len(xm), len(ym)), object)
        for a, x in enumerate(xm):
            for b, y in enumerate(ym):
                inside = any(r[0] < x < r[1] and r[2] < y < r[3] for r in carve)
                c = (min(nx - 1, int(x // cx)), min(ny - 1, int(y // cy)))
                key[a, b] = (floor[c], ceil[c]) if inside else None
        self.boxes = []
        for (a0, a1, b0, b1), k in _merge_tiles(key):
            x0, x1, y0, y1 = xs[a0], xs[a1], ys[b0], ys[b1]
            for kind, z0, z1 in ([('maze', 0.0, H)] if k is None else [('step', 0.0, k[0]), ('soffit', k[1], H)]):
                if z1 - z0 > 1e-6:
                    self.boxes.append(Box(kind, np.array([(x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2]),
                                          np.array([x1 - x0, y1 - y0, z1 - z0])))

        # height maps on a fine raster (NaN where solid), for placing obstacles and the start
        g = self.PIX
        gx, gy = np.arange(0, L, g) + g / 2, np.arange(0, W, g) + g / 2
        ia = np.clip(np.searchsorted(xs, gx) - 1, 0, len(xm) - 1)
        ib = np.clip(np.searchsorted(ys, gy) - 1, 0, len(ym) - 1)
        self.floor_map = np.full((len(gy), len(gx)), np.nan)
        self.ceil_map = np.full((len(gy), len(gx)), np.nan)
        for a in range(len(xm)):
            for b in range(len(ym)):
                if key[a, b] is not None:
                    sel = np.ix_(ib == b, ia == a)
                    self.floor_map[sel], self.ceil_map[sel] = key[a, b]
        self.rooms = rooms
        self.cell_profile = {c: (floor[c], ceil[c]) for c in cells}
        self.info = dict(grid=[nx, ny], cell_m=[round(cx, 2), round(cy, 2)], rooms=len(rooms),
                         links=len(self.links), loops=len(self.links) - tree,
                         dead_ends=int(sum(d == 1 for d in degree.values())),
                         sunken=sum(k == 'sunken' for k in kind_of.values()), raised=sum(k == 'raised' for k in kind_of.values()),
                         low_ceilings=int(sum(v < H for v in ceil.values())), corridor_floor_m=round(float(base), 2),
                         floor_map=self.floor_map, ceiling_map=self.ceil_map, map_resolution=np.array(g))

    # -- queries on the height maps
    def _pix(self, x, y):
        g = self.PIX
        return int(np.clip(y // g, 0, self.floor_map.shape[0] - 1)), int(np.clip(x // g, 0, self.floor_map.shape[1] - 1))

    def profile(self, x, y):
        f = self.floor_map[self._pix(x, y)]
        return None if np.isnan(f) else (float(f), float(self.ceil_map[self._pix(x, y)]))

    def footprint_ok(self, lo, hi, prof):
        """Every raster pixel under the rectangle [lo, hi] is carved and has the profile prof."""
        g = self.PIX
        r0, c0 = int(math.ceil(lo[1] / g - 0.5)), int(math.ceil(lo[0] / g - 0.5))
        r1, c1 = int(math.floor(hi[1] / g - 0.5)) + 1, int(math.floor(hi[0] / g - 0.5)) + 1
        if r0 < 0 or c0 < 0 or r1 > self.floor_map.shape[0] or c1 > self.floor_map.shape[1] or r1 <= r0 or c1 <= c0:
            return False
        f, c = self.floor_map[r0:r1, c0:c1], self.ceil_map[r0:r1, c0:c1]
        return bool(np.all(f == prof[0]) and np.all(c == prof[1]))

    def run(self, x, y, axis):
        """The carved interval through (x, y) along axis (0 x, 1 y) on the raster: (lo, hi) in m."""
        g = self.PIX
        r, c = self._pix(x, y)
        line = ~np.isnan(self.floor_map[r, :] if axis == 0 else self.floor_map[:, c])
        k = c if axis == 0 else r
        if not line[k]:
            return None
        lo, hi = k, k
        while lo > 0 and line[lo - 1]:
            lo -= 1
        while hi < len(line) - 1 and line[hi + 1]:
            hi += 1
        return lo * g, (hi + 1) * g

    def _random_point(self, rng, rooms_only=False):
        if rooms_only:
            if not self.rooms:
                return None
            c = sorted(self.rooms)[int(rng.integers(len(self.rooms)))]
            return np.array([rng.uniform(c[0] * self.cx, (c[0] + 1) * self.cx), rng.uniform(c[1] * self.cy, (c[1] + 1) * self.cy)])
        rr, cc = np.nonzero(~np.isnan(self.floor_map))
        k = int(rng.integers(len(rr)))
        return (np.array([cc[k], rr[k]]) + 0.5) * self.PIX

    # -- obstacles
    def sample(self, kind, rng, p):
        """One large obstacle that forces a height choice, as a list of boxes, or None.

        hurdle  a wall across a corridor from the floor up, leaving >= 2 m above it: fly over
        lintel  a wall across a corridor from the ceiling down, leaving >= 2 m under it: fly under
        bar     a thick bar across a corridor at mid height, >= 1.8 m free above and below: over or under
        shelf   a wide slab out of a chamber wall at mid height, hiding the floor under it from above"""
        xy = self._random_point(rng, rooms_only=kind == 'shelf')
        if xy is None:
            return None
        prof = self.profile(*xy)
        if prof is None:
            return None
        f, top = prof
        gap = top - f
        if kind in ('hurdle', 'lintel', 'bar'):
            along = int(rng.integers(2))                # the obstacle spans the passage along this axis
            span = self.run(xy[0], xy[1], along)
            if span is None or span[1] - span[0] > 8.0:
                return None
            t = rng.uniform(0.4, 1.5)                   # thickness along the passage
            if kind == 'hurdle':
                if gap < 3.5:
                    return None
                z0, z1 = f, f + rng.uniform(1.5, gap - 2.0)
            elif kind == 'lintel':
                if gap < 3.5:
                    return None
                z0, z1 = f + rng.uniform(2.0, gap - 1.5), top
            else:
                if gap < 5.0:
                    return None
                h = rng.uniform(1.0, min(2.5, gap - 3.6))
                z0 = rng.uniform(f + 1.8, top - 1.8 - h)
                z1 = z0 + h
            c, size = np.zeros(3), np.zeros(3)
            c[along], size[along] = (span[0] + span[1]) / 2, span[1] - span[0]
            c[1 - along], size[1 - along] = xy[1 - along], t
            c[2], size[2] = (z0 + z1) / 2, z1 - z0
        elif kind == 'shelf':
            if gap < 4.5:
                return None
            along = int(rng.integers(2))                # the wall it comes out of runs along this axis
            span = self.run(xy[0], xy[1], 1 - along)
            run = self.run(xy[0], xy[1], along)
            if span is None or run is None:
                return None
            d = rng.uniform(2.0, min(5.0, 0.6 * (span[1] - span[0])))
            l = rng.uniform(min(3.0, run[1] - run[0]), run[1] - run[0])
            side = span[0] if rng.random() < 0.5 else span[1]
            c, size = np.zeros(3), np.zeros(3)
            c[1 - along] = side + d / 2 if side == span[0] else side - d / 2
            c[along] = rng.uniform(run[0] + l / 2, run[1] - l / 2)
            c[2] = rng.uniform(f + 2.0, top - 2.0)
            size[1 - along], size[along], size[2] = d, l, 0.3
        else:
            raise ValueError(kind)
        lo, hi = c[:2] - size[:2] / 2, c[:2] + size[:2] / 2
        return [Box(kind, c, size)] if self.footprint_ok(lo, hi, prof) else None

    def pick_start(self, rng, p, cfree):
        """A configuration-space-free cell start_clearance above its local floor (or below its local ceiling), the
        nearest the check grid allows; drawn at random over the whole maze."""
        cr = p.check_resolution
        zs, ys, xs = np.nonzero(cfree)
        pts = (np.stack([xs, ys, zs], 1) + 0.5) * cr
        rr = np.clip((pts[:, 1] // self.PIX).astype(int), 0, self.floor_map.shape[0] - 1)
        cc = np.clip((pts[:, 0] // self.PIX).astype(int), 0, self.floor_map.shape[1] - 1)
        f, c = self.floor_map[rr, cc], self.ceil_map[rr, cc]
        target = f + p.start_clearance if p.start == 'floor' else c - p.start_clearance
        err = np.abs(pts[:, 2] - target)
        ok = np.nonzero(np.nan_to_num(err, nan=np.inf) <= np.nanmin(err) + 0.5 * cr)[0]
        return pts[ok[int(rng.integers(len(ok)))]]


def _maze(rng, room, p):
    return _Maze(rng, room, p)


def _merge_tiles(key):
    """Greedy merge of a 2D grid of keys into rectangles of equal key (keys equal to the carved full-height profile
    are still emitted; their box list may be empty). Yields ((a0, a1, b0, b1), key) with half-open tile ranges."""
    na, nb = key.shape
    used = np.zeros((na, nb), bool)
    for a in range(na):
        for b in range(nb):
            if used[a, b]:
                continue
            k = key[a, b]
            b1 = b
            while b1 + 1 < nb and not used[a, b1 + 1] and key[a, b1 + 1] == k:
                b1 += 1
            a1 = a
            while a1 + 1 < na and all(not used[a1 + 1, bb] and key[a1 + 1, bb] == k for bb in range(b, b1 + 1)):
                a1 += 1
            used[a:a1 + 1, b:b1 + 1] = True
            yield (a, a1 + 1, b, b1 + 1), k


def split_of(seed):
    return next(name for name, r in SPLITS.items() if seed in r)


# ---- obstacles -----------------------------------------------------------------------------------------------

def _shell(room, t):
    L, W, H = room
    return [Box('room', np.array([L / 2, W / 2, -t / 2]), np.array([L + 2 * t, W + 2 * t, t])),        # floor
            Box('room', np.array([L / 2, W / 2, H + t / 2]), np.array([L + 2 * t, W + 2 * t, t])),     # ceiling
            Box('room', np.array([-t / 2, W / 2, H / 2]), np.array([t, W + 2 * t, H])),
            Box('room', np.array([L + t / 2, W / 2, H / 2]), np.array([t, W + 2 * t, H])),
            Box('room', np.array([L / 2, -t / 2, H / 2]), np.array([L, t, H])),
            Box('room', np.array([L / 2, W + t / 2, H / 2]), np.array([L, t, H]))]


def _sample(kind, rng, room, p, state):
    """One candidate obstacle (a list of boxes), or None when there is nowhere left to put it."""
    L, W, H = room
    if kind == 'pillar':
        grid = state['pillar_grid']
        if not grid:
            return None
        k = int(rng.integers(len(grid)))
        state['pillar_pick'] = k  # removed from the grid only if the pillar is accepted
        w = rng.uniform(0.4, 1.0)
        x, y = grid[k] + rng.uniform(-0.25, 0.25, 2)
        return [Box('pillar', np.array([x, y, H / 2]), np.array([w, w, H]))]
    if kind == 'block':  # anchored: on the floor, hanging from the ceiling, or sticking out of a side wall
        where = rng.choice(['floor', 'floor', 'ceiling', 'wall'])
        if where == 'wall':
            return [_from_wall('block', rng, room, depth=(1.0, 2.5), length=(1.0, 3.0), thick=(1.0, 2.5))]
        s = np.array([rng.uniform(1.0, 3.0), rng.uniform(1.0, 3.0), rng.uniform(1.0, min(3.0, H / 2))])
        if where == 'ceiling':
            s[2] = rng.uniform(1.0, min(2.5, H / 3))
        yaw = rng.uniform(0, math.pi) if (p.yaw_blocks and where == 'floor') else 0.0
        rx, ry = _xy_reach(s, yaw)
        z = s[2] / 2 if where == 'floor' else H - s[2] / 2
        return [Box('block', np.array([rng.uniform(rx, L - rx), rng.uniform(ry, W - ry), z]), s, yaw)]
    if kind == 'floating':
        s = rng.uniform(1.0, 2.5, 3)
        yaw = rng.uniform(0, math.pi) if p.yaw_blocks else 0.0
        rx, ry = _xy_reach(s, yaw)
        if H - s[2] < 4.0:
            return None  # needs room above and below
        z = rng.uniform(2.0 + s[2] / 2, H - 2.0 - s[2] / 2)
        return [Box('floating', np.array([rng.uniform(rx, L - rx), rng.uniform(ry, W - ry), z]), s, yaw)]
    if kind == 'beam':  # wall to wall, or cantilevered from one wall
        along = int(rng.integers(2))
        t = rng.uniform(0.3, 0.5)
        span = room[along] if rng.random() < 0.6 else room[along] * rng.uniform(0.3, 0.6)
        c = np.array([0.0, 0.0, rng.uniform(1.5, H - 1.0)])
        c[along] = span / 2 if rng.random() < 0.5 else room[along] - span / 2
        c[1 - along] = rng.uniform(1.0, room[1 - along] - 1.0)
        size = np.array([t, t, t])
        size[along] = span
        return [Box('beam', c, size)]
    if kind == 'ledge':  # a thin slab sticking out of a side wall
        if H < 4.0:
            return None
        return [_from_wall('ledge', rng, room, depth=(1.5, 4.0), length=(2.0, 6.0), thick=(0.2, 0.2), z=(1.5, H - 1.5))]
    if kind == 'wall':
        return _partial_wall(rng, room, group=int(rng.integers(1 << 30)))
    raise ValueError(kind)


def _from_wall(kind, rng, room, depth, length, thick, z=None):
    """A box touching one of the four side walls: `depth` out from the wall, `length` along it."""
    L, W, H = room
    side = int(rng.integers(4))                      # 0: x=0, 1: x=L, 2: y=0, 3: y=W
    normal, along = (0, 1) if side < 2 else (1, 0)
    d, l, t = rng.uniform(*depth), rng.uniform(*length), rng.uniform(*thick)
    size, c = np.zeros(3), np.zeros(3)
    size[normal], size[along], size[2] = d, l, t
    c[normal] = d / 2 if side in (0, 2) else room[normal] - d / 2
    c[along] = rng.uniform(l / 2, room[along] - l / 2)
    c[2] = rng.uniform(*(z or (t / 2, H - t / 2)))
    return Box(kind, c, size)


def _pillar_grid(rng, room, p, margin=2.5):
    """Candidate pillar spots: a regular grid with a random pitch and offset, kept margin from the walls."""
    pitch = rng.uniform(*p.pillar_spacing)
    pts = []
    for axis in (0, 1):
        n = max(1, int((room[axis] - 2 * margin) // pitch) + 1)
        used = (n - 1) * pitch
        off = margin + rng.uniform(0, max(0.0, room[axis] - 2 * margin - used))
        pts.append(off + pitch * np.arange(n))
    return [np.array([x, y]) for x in pts[0] for y in pts[1]]


def _xy_reach(size, yaw):
    return (abs(math.cos(yaw)) * size[0] / 2 + abs(math.sin(yaw)) * size[1] / 2,
            abs(math.sin(yaw)) * size[0] / 2 + abs(math.cos(yaw)) * size[1] / 2)


def _aabb(b):
    rx, ry = _xy_reach(b.size, b.yaw)
    half = np.array([rx, ry, b.size[2] / 2])
    return b.center - half, b.center + half


def _clear_of(new, others, gap):
    """True if every box in `new` is at least `gap` from every box in `others` (bounding boxes, so yawed boxes are
    treated a little larger than they are)."""
    for a in new:
        alo, ahi = _aabb(a)
        for b in others:
            blo, bhi = _aabb(b)
            sep = np.maximum(0.0, np.maximum(blo - ahi, alo - bhi))
            if np.linalg.norm(sep) < gap:
                return False
    return True


def _partial_wall(rng, room, t=0.3, group=0):
    """A full-height dividing wall across the room, with large openings: either it stops short of one side wall
    (a 3-6 m gap) or it runs wall to wall with two doorways. Doorways are 3-6 m wide and at least 2.5 m tall (up
    to the ceiling less 1 m); a high window may be added."""
    across = int(rng.integers(2))                   # the wall runs along this axis: 0 x, 1 y
    normal = 1 - across
    length, H = room[across], room[2]
    pos = rng.uniform(0.35, 0.65) * room[normal]
    holes = []
    if rng.random() < 0.5:                          # stops short of one end
        gap = rng.uniform(3.0, 6.0)
        holes.append((0.0, gap, 0.0, H) if rng.random() < 0.5 else (length - gap, length, 0.0, H))
    else:                                           # two doorways, in different halves of the wall
        for lo, hi in ((0.0, length / 2), (length / 2, length)):
            w = rng.uniform(3.0, min(6.0, hi - lo - 0.5))
            a = rng.uniform(lo + 0.25, hi - 0.25 - w)
            holes.append((a, a + w, 0.0, max(2.5, H - rng.uniform(1.0, 3.0))))
    if H >= 6.0 and rng.random() < 0.5:             # a high window away from the other openings
        w, h = rng.uniform(2.0, 4.0), rng.uniform(1.5, 2.5)
        for _ in range(10):
            a = rng.uniform(0.5, length - w - 0.5)
            if all(a + w + 1.0 < h0 or a > h1 + 1.0 for h0, h1, _, _ in holes):
                z0 = rng.uniform(H / 2, H - h - 0.5)
                holes.append((a, a + w, z0, z0 + h))
                break
    # split the wall plane (along-axis s, height z) into rectangles around the holes, on the holes' grid lines
    ss = sorted({0.0, length, *[h[0] for h in holes], *[h[1] for h in holes]})
    zs = sorted({0.0, H, *[h[2] for h in holes], *[h[3] for h in holes]})
    pieces = []
    for s0, s1 in zip(ss, ss[1:]):
        for z0, z1 in zip(zs, zs[1:]):
            sm, zm = (s0 + s1) / 2, (z0 + z1) / 2
            if any(h[0] < sm < h[1] and h[2] < zm < h[3] for h in holes) or s1 - s0 < 1e-6 or z1 - z0 < 1e-6:
                continue
            c, size = np.zeros(3), np.zeros(3)
            c[across], c[normal], c[2] = sm, pos, zm
            size[across], size[normal], size[2] = s1 - s0, t, z1 - z0
            pieces.append(Box('wall', c, size, group=group))
    return pieces


def _pick_start(rng, p, room, cfree):
    cr = p.check_resolution
    z = p.start_clearance if p.start == 'floor' else room[2] - p.start_clearance
    if p.start not in ('floor', 'ceiling'):
        raise ValueError("start must be 'floor' or 'ceiling'")
    # the check-grid layer nearest the wanted height that has room for the drone; the start sits at its centre,
    # so it is free in configuration space by construction
    centres = (np.arange(cfree.shape[0]) + 0.5) * cr
    for layer in np.argsort(np.abs(centres - z), kind='stable'):
        ys, xs = np.nonzero(cfree[layer])
        if len(xs):
            k = rng.integers(len(xs))
            return np.array([(xs[k] + 0.5) * cr, (ys[k] + 0.5) * cr, centres[layer]])
    raise ValueError('no free start cell in the room')


# ---- grids ---------------------------------------------------------------------------------------------------

def _rasterize(boxes, origin, room, res, inflate):
    """[z, y, x] grid over [origin, origin + room]: True where a cell centre lies inside a box grown by `inflate` on
    every side (in the box's own frame; for a yawed box that is the box's footprint, not a circle)."""
    shape = np.ceil(room / res).astype(int)
    grid = np.zeros(shape[::-1], bool)
    for b in boxes:
        half = np.asarray(b.size) / 2 + inflate
        reach = np.abs(np.cos(b.yaw)) * half[0] + np.abs(np.sin(b.yaw)) * half[1], \
            np.abs(np.sin(b.yaw)) * half[0] + np.abs(np.cos(b.yaw)) * half[1], half[2]
        lo = np.clip(np.floor((b.center - reach - origin) / res - 0.5).astype(int), 0, shape)
        hi = np.clip(np.ceil((b.center + reach - origin) / res + 0.5).astype(int), 0, shape)
        if np.any(hi <= lo):
            continue
        xs = origin[0] + (np.arange(lo[0], hi[0]) + 0.5) * res - b.center[0]
        ys = origin[1] + (np.arange(lo[1], hi[1]) + 0.5) * res - b.center[1]
        zs = origin[2] + (np.arange(lo[2], hi[2]) + 0.5) * res - b.center[2]
        c, s = math.cos(b.yaw), math.sin(b.yaw)
        lx = c * xs[None, :] + s * ys[:, None]           # world offset rotated into the box frame
        ly = -s * xs[None, :] + c * ys[:, None]
        foot = (np.abs(lx) <= half[0]) & (np.abs(ly) <= half[1])
        inside = (np.abs(zs) <= half[2])[:, None, None] & foot[None]
        grid[lo[2]:hi[2], lo[1]:hi[1], lo[0]:hi[0]] |= inside
    return grid


def _flood(mask, start):
    """Cells of `mask` 6-connected to `start` (a [z, y, x] index), by frontier expansion."""
    out = np.zeros_like(mask)
    if not mask[start]:
        return out
    shape = np.array(mask.shape)
    out[start] = True
    frontier = np.array([start])
    steps = np.array([[1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0], [0, 0, 1], [0, 0, -1]])
    while len(frontier):
        nb = (frontier[:, None, :] + steps[None]).reshape(-1, 3)
        nb = nb[np.all((nb >= 0) & (nb < shape), axis=1)]
        nb = nb[mask[nb[:, 0], nb[:, 1], nb[:, 2]] & ~out[nb[:, 0], nb[:, 1], nb[:, 2]]]
        if len(nb) == 0:
            break
        nb = np.unique(nb, axis=0)
        out[nb[:, 0], nb[:, 1], nb[:, 2]] = True
        frontier = nb
    return out


def _dilate6(mask):
    out = mask.copy()
    for axis in range(3):
        for shift in (1, -1):
            out |= np.roll(mask, shift, axis=axis) & _edge_ok(mask.shape, axis, shift)
    return out


def _edge_ok(shape, axis, shift):
    ok = np.ones(shape, bool)
    idx = [slice(None)] * 3
    idx[axis] = 0 if shift == 1 else -1
    ok[tuple(idx)] = False
    return ok


def _jsonable(d):
    return {k: (list(v) if isinstance(v, tuple) else {kk: list(vv) for kk, vv in v.items()} if isinstance(v, dict) else v)
            for k, v in d.items()}
