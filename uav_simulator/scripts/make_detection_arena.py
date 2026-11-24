#!/usr/bin/env python3
"""Worlds for testing moving-object detection: an open textured arena with one target that is a person or a box (same
0.5 x 0.5 x 1.8 m body, so ray sensors see them alike; only the camera sees the difference), static or walking a set
heading relative to the drone's test flight.

The drone (MALT weave_flight.py) starts at (0, 0, 1.2) and, from sim time T_START, flies along +x at 1 m/s weaving
+-0.8 m. Each heading is timed (the target's animation starts at sim time 0) so the target meets the drone at a set
place: 10 s into the weave, when the drone is at x = 10 m.

  make_detection_arena.py --arena                          # the arena model (once; uses corridor_textured's textures)
  make_detection_arena.py --target person --heading cross  # worlds/detection_arena/person_cross.world
  make_detection_arena.py --all                            # every target x heading
  make_detection_arena.py --random [--seed 1 --walk 0.8]   # worlds/detection_arena/person_random<seed>.world: following
  make_detection_arena.py --crowd [--seed 1]               # worlds/detection_arena/crowd<seed>.world: five coloured
                                                           # person-sized capsules (target_red, _orange, _yellow, _green,
                                                           # _blue), each on its own random walk at 0.6-1.0 m/s

Headings (relative to the drone's +x travel):
  static            standing 1.5 m off the path at x 12
  head_on           walking at the drone (-x), 2 m off the path, passing at x 10
  same_dir          walking ahead of it (+x), 2 m off the path, overtaken at x 10
  cross             crossing the path left to right (+y), 4 m ahead of the drone (at x 14)
  cross45_approach  crossing diagonally towards the drone, 4 m ahead (at x 14)
  cross45_recede    crossing diagonally away from it, 3 m ahead (at x 13)
"""
import argparse
import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SIM = os.path.normpath(os.path.join(HERE, '..'))
MODEL = os.path.join(SIM, 'models', 'detection_arena')
WORLDS = os.path.join(SIM, 'worlds', 'detection_arena')
X0, X1, Y0, Y1, H = -6.0, 28.0, -12.0, 12.0, 3.0
T_START, SPEED, WALK = 15.0, 1.0, 0.5   # weave start (sim s), drone speed, target speed
MEET = 10.0                              # s into the weave
TOTAL = 50.0                             # s the target walks one way (its path is 25 m)
HEADINGS = ['static', 'head_on', 'same_dir', 'cross', 'cross45_approach', 'cross45_recede']


def mat(name):
    return ('<material><script><uri>model://corridor_textured/materials/scripts</uri>'
            '<uri>model://corridor_textured/materials/textures</uri><name>%s</name></script></material>' % name)


def box(name, size, pose, material=None, collide=True):
    p = ' '.join('%.3f' % v for v in pose)
    s = '%.3f %.3f %.3f' % tuple(size)
    out = '<visual name="%s_v"><pose>%s</pose><geometry><box><size>%s</size></box></geometry>%s</visual>' % (name, p, s, mat(material) if material else '')
    if collide: out += '<collision name="%s_c"><pose>%s</pose><geometry><box><size>%s</size></box></geometry></collision>' % (name, p, s)
    return out


def make_arena(seed=3):
    rng = np.random.default_rng(seed)
    parts = []
    T = 0.2
    for name, size, c, axis in (('n', (X1 - X0, T, H), ((X0 + X1) / 2, Y1, H / 2), 'x'), ('s', (X1 - X0, T, H), ((X0 + X1) / 2, Y0, H / 2), 'x'),
                                ('w', (T, Y1 - Y0, H), (X0, 0, H / 2), 'y'), ('e', (T, Y1 - Y0, H), (X1, 0, H / 2), 'y')):
        parts.append('<collision name="wall_%s"><pose>%.2f %.2f %.2f 0 0 0</pose><geometry><box><size>%.2f %.2f %.2f</size></box></geometry></collision>'
                     % (name, *c, *size))
        lo, hi = (X0, X1) if axis == 'x' else (Y0, Y1)
        for k, a in enumerate(np.arange(lo, hi, 3.0)):
            b = min(a + 3.0, hi); m = (a + b) / 2
            sz, pos = ((b - a, T, H), (m, c[1], H / 2)) if axis == 'x' else ((T, b - a, H), (c[0], m, H / 2))
            parts.append(box('wall_%s_%d' % (name, k), sz, (*pos, 0, 0, 0), 'Corridor/Wall%d' % rng.integers(6), collide=False))
    for i, x in enumerate(np.arange(X0, X1, 3.0)):
        for j, y in enumerate(np.arange(Y0, Y1, 3.0)):
            parts.append(box('floor_%d_%d' % (i, j), (3.0, 3.0, 0.01), (x + 1.5, y + 1.5, 0.005, 0, 0, 0), 'Corridor/Floor%d' % rng.integers(3), collide=False))
    for k in range(16):  # paintings on the walls, for texture all round
        side = rng.integers(4)
        w, h = rng.uniform(1.0, 1.8), rng.uniform(0.7, 1.2)
        if side < 2:
            x, y, yaw = rng.uniform(X0 + 2, X1 - 2), (Y1 - 0.12 if side == 0 else Y0 + 0.12), (-np.pi / 2 if side == 0 else np.pi / 2)
        else:
            x, y, yaw = (X1 - 0.12 if side == 2 else X0 + 0.12), rng.uniform(Y0 + 2, Y1 - 2), (np.pi if side == 2 else 0.0)
        parts.append(box('painting_%d' % k, (0.02, w, h), (x, y, rng.uniform(1.0, 2.0), 0, 0, yaw), 'Corridor/Painting%02d' % (k % 16), collide=False))
    os.makedirs(MODEL, exist_ok=True)
    open(os.path.join(MODEL, 'detection_arena.sdf'), 'w').write(
        '<?xml version="1.0"?>\n<!-- generated by uav_simulator/scripts/make_detection_arena.py -->\n'
        '<sdf version="1.6"><model name="detection_arena"><static>true</static><link name="link">\n%s\n</link></model></sdf>\n' % '\n'.join(parts))
    open(os.path.join(MODEL, 'model.config'), 'w').write(
        '<?xml version="1.0"?>\n<model><name>detection_arena</name><version>1.0</version><sdf version="1.6">detection_arena.sdf</sdf>\n'
        '<description>Open textured arena x [%g, %g], y [%g, %g], walls %g m, for moving-object detection tests.</description></model>\n' % (X0, X1, Y0, Y1, H))


def target_path(heading):
    """(start, end) of a straight 25 m walk timed to meet the drone, or a fixed point for static."""
    t_meet = T_START + MEET            # sim time of the encounter
    def line(meet_xy, v):
        v = np.asarray(v, float); meet = np.asarray(meet_xy, float)
        return meet - v * t_meet, meet + v * (TOTAL - t_meet)
    d = WALK / np.sqrt(2)
    return {'static': None,
            'head_on': line((10.0, 2.0), (-WALK, 0)),
            'same_dir': line((10.0, 2.0), (WALK, 0)),
            'cross': line((14.0, 0.0), (0, WALK)),
            'cross45_approach': line((14.0, 0.0), (-d, d)),
            'cross45_recede': line((13.0, 0.0), (d, d))}[heading]


def random_route(seed, n=40, x=(-2.0, 24.0), y=(-8.0, 8.0), start=(5.0, 0.0)):
    """A wandering walk: legs of 3-8 m, turns of at most 100 deg, kept inside x, y (2+ m from the arena walls)."""
    rng = np.random.default_rng(seed)
    pts, hd = [np.array(start)], 0.0
    while len(pts) < n:
        for _ in range(50):
            h = hd + np.radians(rng.uniform(-100, 100)); L = rng.uniform(3.0, 8.0)
            q = pts[-1] + L * np.array([np.cos(h), np.sin(h)])
            if x[0] <= q[0] <= x[1] and y[0] <= q[1] <= y[1]: break
        else:
            h = np.arctan2(-pts[-1][1], 11.0 - pts[-1][0]); q = pts[-1] + 3.0 * np.array([np.cos(h), np.sin(h)])  # boxed in: head for the middle
        pts.append(q); hd = h
    return pts


def make_random_world(seed, walk):
    """The person on a random looping walk (it returns to the start and repeats), starting 5 m ahead of the drone."""
    os.makedirs(WORLDS, exist_ok=True)
    pts = random_route(seed)
    visual = '<visual name="visual"><pose>0 0 -0.02 0 0 1.57</pose><geometry><mesh><uri>model://person/meshes/walking.dae</uri></mesh></geometry></visual>'
    body = '<collision name="body"><pose>0 0 0.9 0 0 0</pose><geometry><box><size>0.5 0.5 1.8</size></box></geometry></collision>'
    wps = ''.join('<waypoint>%.3f %.3f 0</waypoint>' % (q[0], q[1]) for q in pts)
    model = ('<model name="target"><pose>%.3f %.3f 0 0 0 0</pose><link name="link"><gravity>false</gravity>%s%s</link>'
             '<plugin name="obstacle_motion" filename="libobstaclePathPlugin.so"><loop>1</loop><velocity>%.3f</velocity>'
             '<angular_velocity>2.0</angular_velocity><path>%s</path></plugin></model>' % (pts[0][0], pts[0][1], visual, body, walk, wps))
    world = ('<?xml version="1.0"?>\n<!-- person on a random walk (seed %d, %.2f m/s): generated by make_detection_arena.py --random -->\n'
             '<sdf version="1.6"><world name="detection_person_random%d">\n'
             '<include><uri>model://sun</uri></include><include><uri>model://ground_plane</uri></include>\n'
             '<include><uri>model://detection_arena</uri></include>\n%s\n</world></sdf>\n' % (seed, walk, seed, model))
    out = os.path.join(WORLDS, 'person_random%d.world' % seed)
    open(out, 'w').write(world)
    return out


CROWD = ['red', 'orange', 'yellow', 'green', 'blue']


def capsule(color):
    """Person-sized rounded cylinder, 0.5 m wide and 1.75 m tall: a cylinder with a sphere at each end (Gazebo 11 has no
    capsule shape). Same shapes for the visual and the collision, so the lidar sees what the camera sees."""
    mat = '<material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/%s</name></script></material>' % color.capitalize()
    shapes = (('bottom', '<sphere><radius>0.25</radius></sphere>', 0.25), ('body', '<cylinder><radius>0.25</radius><length>1.25</length></cylinder>', 0.875),
              ('top', '<sphere><radius>0.25</radius></sphere>', 1.5))
    out = ''
    for n, g, z in shapes:
        out += '<visual name="%s_v"><pose>0 0 %.3f 0 0 0</pose><geometry>%s</geometry>%s</visual>' % (n, z, g, mat)
        out += '<collision name="%s_c"><pose>0 0 %.3f 0 0 0</pose><geometry>%s</geometry></collision>' % (n, z, g)
    return out


def make_crowd_world(seed):
    os.makedirs(WORLDS, exist_ok=True)
    rng = np.random.default_rng(1000 + seed)
    models = []
    for k, color in enumerate(CROWD):
        start = (rng.uniform(2.0, 22.0), rng.uniform(-7.0, 7.0))
        pts = random_route(seed * 10 + k, start=start)
        walk = rng.uniform(0.6, 1.0)
        wps = ''.join('<waypoint>%.3f %.3f 0</waypoint>' % (q[0], q[1]) for q in pts)
        models.append('<model name="target_%s"><pose>%.3f %.3f 0 0 0 0</pose><link name="link"><gravity>false</gravity>%s</link>'
                      '<plugin name="obstacle_motion_%s" filename="libobstaclePathPlugin.so"><loop>1</loop><velocity>%.3f</velocity>'
                      '<angular_velocity>2.0</angular_velocity><path>%s</path></plugin></model>' % (color, pts[0][0], pts[0][1], capsule(color), color, walk, wps))
    world = ('<?xml version="1.0"?>\n<!-- five coloured capsules on random walks (seed %d): generated by make_detection_arena.py --crowd -->\n'
             '<sdf version="1.6"><world name="detection_crowd%d">\n'
             '<include><uri>model://sun</uri></include><include><uri>model://ground_plane</uri></include>\n'
             '<include><uri>model://detection_arena</uri></include>\n%s\n</world></sdf>\n' % (seed, seed, '\n'.join(models)))
    out = os.path.join(WORLDS, 'crowd%d.world' % seed)
    open(out, 'w').write(world)
    return out


def make_world(target, heading):
    os.makedirs(WORLDS, exist_ok=True)
    if target == 'person':
        visual = '<visual name="visual"><pose>0 0 -0.02 0 0 1.57</pose><geometry><mesh><uri>model://person/meshes/walking.dae</uri></mesh></geometry></visual>'
    else:
        visual = '<visual name="visual"><pose>0 0 0.9 0 0 0</pose><geometry><box><size>0.5 0.5 1.8</size></box></geometry>%s</visual>' % mat('Corridor/Plywood')
    body = '<collision name="body"><pose>0 0 0.9 0 0 0</pose><geometry><box><size>0.5 0.5 1.8</size></box></geometry></collision>'
    path = target_path(heading)
    if path is None:
        model = '<model name="target"><static>true</static><pose>12.0 1.5 0 0 0 0</pose><link name="link">%s%s</link></model>' % (visual, body)
    else:
        a, b = path
        model = ('<model name="target"><pose>%.3f %.3f 0 0 0 0</pose><link name="link"><gravity>false</gravity>%s%s</link>'
                 '<plugin name="obstacle_motion" filename="libobstaclePathPlugin.so"><loop>0</loop><velocity>%.3f</velocity>'
                 '<angular_velocity>3.0</angular_velocity><path><waypoint>%.3f %.3f 0</waypoint><waypoint>%.3f %.3f 0</waypoint></path></plugin></model>'
                 % (a[0], a[1], visual, body, WALK, a[0], a[1], b[0], b[1]))
    world = ('<?xml version="1.0"?>\n<!-- %s, %s: generated by make_detection_arena.py -->\n<sdf version="1.6"><world name="detection_%s_%s">\n'
             '<include><uri>model://sun</uri></include><include><uri>model://ground_plane</uri></include>\n'
             '<include><uri>model://detection_arena</uri></include>\n%s\n</world></sdf>\n' % (target, heading, target, heading, model))
    out = os.path.join(WORLDS, '%s_%s.world' % (target, heading))
    open(out, 'w').write(world)
    return out


if __name__ == '__main__':
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--arena', action='store_true'); ap.add_argument('--all', action='store_true')
    ap.add_argument('--target', choices=['person', 'box']); ap.add_argument('--heading', choices=HEADINGS)
    ap.add_argument('--crowd', action='store_true'); ap.add_argument('--random', action='store_true'); ap.add_argument('--seed', type=int, default=1); ap.add_argument('--walk', type=float, default=0.8)
    a = ap.parse_args()
    if a.random: print('Wrote', make_random_world(a.seed, a.walk))
    if a.crowd: print('Wrote', make_crowd_world(a.seed))
    if a.arena or a.all: make_arena(); print('Wrote', MODEL)
    if a.all:
        for t in ('person', 'box'):
            for h in HEADINGS: print('Wrote', make_world(t, h))
    elif a.target and a.heading:
        print('Wrote', make_world(a.target, a.heading))
