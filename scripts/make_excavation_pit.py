#!/usr/bin/env python3
"""A deep excavation pit: a vertical site for exploring from above. The floor (36 x 26 m, z = 0) is where the work is:
an excavator digging and dumping into a truck, people walking, material stacks. Around it 9 m walls, a 4.5 m bench
along the north wall and two horizontal struts across the pit at 7.5 m (as in a braced excavation). A tower crane
stands on the floor (16 m mast, slewing jib) and hoists its load between the floor and above the rim, so it moves up
and down through the whole height of the pit. The drone starts on a deck at the rim (z = 9 m, west wall).

  make_excavation_pit.py [--seed 1]   # worlds/detection_arena/excavation_pit<seed>.world

Machines: the same models and motion as make_excavation_site.py (functions taking the layout PIT), posed by
site_motion.py _layout:=pit, which also publishes their true shapes on /site/hazards for scoring.
Spawn the drone at START (start_detection_arena.launch spawn_x/y/z; MALT site_explore.launch world:=pit does).
"""
import argparse
import os

import numpy as np

from make_detection_arena import WORLDS, box, capsule
from make_excavation_site import PEOPLE, machine_models, safe_route, in_hazard_area

X, Y, WALL_H = (-6.0, 30.0), (-13.0, 13.0), 9.0       # pit interior and wall height
BENCH = dict(y=(10.0, 13.0), x=(-6.0, 30.0), h=4.5)   # terrace along the north wall
STRUTS = [14.0, 24.0]                                 # x of the struts across the pit
STRUT_Z, STRUT_W = 7.5, 0.6
DECK = dict(x=(-6.0, -3.0), y=(-2.0, 2.0), z=WALL_H)  # start deck jutting from the west wall at the rim
START = (-4.5, 0.0, WALL_H + 0.1)

PIT = {
    'excavator': dict(pos=(21.0, -2.0), dig_deg=180.0, dump_deg=270.0, foot=(1.2, 2.2), arm_len=7.5, dig_elev=-16.0, carry_elev=20.0),
    'crane': dict(pos=(8.0, 5.0), radius=6.5, pick_deg=250.0, set_deg=400.0, mast_h=16.0, low=0.3, carry=10.5, slew_rate=0.2,
                  hoist_rate=0.8, jib=True),
    'truck': (21.0, -8.8),
    'berms': [],
    'keep_out': [((0.5 * sum(BENCH['x']), 0.5 * sum(BENCH['y'])), (BENCH['x'][1] - BENCH['x'][0], BENCH['y'][1] - BENCH['y'][0]))],
    'walk_x': (-5.0, 29.0), 'walk_y': (-12.0, 8.5),
}


def pit_structure():
    """Walls, bench, struts and deck as one static model."""
    t = 1.0
    cx, cy = 0.5 * sum(X), 0.5 * sum(Y)
    lx, ly = X[1] - X[0], Y[1] - Y[0]
    parts = [
        box('wall_w', (t, ly + 2 * t, WALL_H), (X[0] - t / 2, cy, WALL_H / 2, 0, 0, 0), 'Corridor/Wall0'),
        box('wall_e', (t, ly + 2 * t, WALL_H), (X[1] + t / 2, cy, WALL_H / 2, 0, 0, 0), 'Corridor/Wall1'),
        box('wall_s', (lx, t, WALL_H), (cx, Y[0] - t / 2, WALL_H / 2, 0, 0, 0), 'Corridor/Wall2'),
        box('wall_n', (lx, t, WALL_H), (cx, Y[1] + t / 2, WALL_H / 2, 0, 0, 0), 'Corridor/Wall3'),
        box('bench', (BENCH['x'][1] - BENCH['x'][0], BENCH['y'][1] - BENCH['y'][0], BENCH['h']),
            (0.5 * sum(BENCH['x']), 0.5 * sum(BENCH['y']), BENCH['h'] / 2, 0, 0, 0), 'Corridor/Floor1'),
        box('deck', (DECK['x'][1] - DECK['x'][0], DECK['y'][1] - DECK['y'][0], 0.4),
            (0.5 * sum(DECK['x']), 0.5 * sum(DECK['y']), DECK['z'] - 0.2, 0, 0, 0), 'Corridor/Plywood'),
    ]
    for k, x in enumerate(STRUTS):
        parts.append(box('strut_%d' % k, (STRUT_W, ly, STRUT_W), (x, cy, STRUT_Z, 0, 0, 0), 'Corridor/Painting05'))
        for y in (Y[0] + 0.3, Y[1] - 0.3):   # walers: the beams along the wall the struts bear on
            parts.append(box('waler_%d_%d' % (k, int(y > 0)), (3.0, 0.5, 0.6), (x, y, STRUT_Z, 0, 0, 0), 'Corridor/Wall5'))
    return '<model name="pit"><static>true</static><pose>0 0 0 0 0 0</pose><link name="link">%s</link></model>' % ''.join(parts)


def make_pit(seed):
    rng = np.random.default_rng(seed)
    S = PIT
    models = [pit_structure()] + machine_models(S)
    e = S['excavator']; L = e['arm_len']
    dig = np.array(e['pos']) + (e['foot'][0] + L * np.cos(np.radians(e['dig_elev'])) + 1.6) * np.array([np.cos(np.radians(e['dig_deg'])), np.sin(np.radians(e['dig_deg']))])
    models.append('<model name="spoil"><static>true</static><pose>%.2f %.2f 0 0 0 0.4</pose><link name="link">%s</link></model>'
                  % (dig[0], dig[1] + 2.5, box('pile', (2.5, 2.0, 0.9), (0, 0, 0.45, 0, 0, 0), 'Corridor/Floor1')))
    placed = []
    while len(placed) < 6:
        q = np.array([rng.uniform(*S['walk_x']), rng.uniform(*S['walk_y'])])
        if in_hazard_area(q, pad=1.5, S=S) or any(np.hypot(*(q - p)) < 4 for p in placed): continue
        placed.append(q)
        w, d, h = rng.uniform(0.8, 2.0), rng.uniform(0.8, 2.0), rng.uniform(0.8, 2.4)
        models.append('<model name="stack_%d"><static>true</static><pose>%.2f %.2f 0 0 0 %.2f</pose><link name="link">%s</link></model>'
                      % (len(placed), q[0], q[1], rng.uniform(0, np.pi), box('s', (w, d, h), (0, 0, h / 2, 0, 0, 0), 'Corridor/Plywood')))
    for k, color in enumerate(PEOPLE + ['green']):
        while True:
            start = np.array([rng.uniform(*S['walk_x']), rng.uniform(*S['walk_y'])])
            if not in_hazard_area(start, S=S) and all(np.hypot(*(start - p)) > 1.5 for p in placed): break
        pts = safe_route(rng, start, S=S)
        wps = ''.join('<waypoint>%.3f %.3f 0</waypoint>' % (q[0], q[1]) for q in pts)
        models.append('<model name="person_%s"><pose>%.3f %.3f 0 0 0 0</pose><link name="link"><gravity>false</gravity>%s</link>'
                      '<plugin name="walk_%s" filename="libobstaclePathPlugin.so"><loop>1</loop><velocity>%.3f</velocity>'
                      '<angular_velocity>2.0</angular_velocity><path>%s</path></plugin></model>'
                      % (color, pts[0][0], pts[0][1], capsule(color), color, rng.uniform(0.6, 1.0), wps))
    world = ('<?xml version="1.0"?>\n<!-- excavation pit (seed %d): generated by make_excavation_pit.py; machines driven by site_motion.py _layout:=pit;'
             ' drone start %s -->\n<sdf version="1.6"><world name="excavation_pit%d">\n'
             '<include><uri>model://sun</uri></include><include><uri>model://ground_plane</uri></include>\n%s\n</world></sdf>\n'
             % (seed, START, seed, '\n'.join(models)))
    os.makedirs(WORLDS, exist_ok=True)
    out = os.path.join(WORLDS, 'excavation_pit%d.world' % seed)
    open(out, 'w').write(world)
    return out


if __name__ == '__main__':
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--seed', type=int, default=1)
    print('Wrote', make_pit(ap.parse_args().seed))
