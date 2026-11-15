#!/usr/bin/env python3
"""An excavation site in the detection arena (34 x 24 m, 3 m walls): an excavator digging and dumping over and over, a
crane slewing a hanging load back and forth on its cable, people walking, and static clutter (a dump truck, a spoil
pile, material stacks, and three 2.6 m spoil berms that hide what is behind them, so the site has to be flown around). For exploring among repetitive dynamic hazards (MALT site_explore.launch).

The moving machines are kinematic models posed every step by site_motion.py (same geometry: SITE below), which also
publishes their true shape as capsules on /site/hazards for scoring. The people walk with the obstacle path plugin and
keep out of the excavator's swing sector and the crane's slewing arc, as site rules would have them.

  make_excavation_site.py [--seed 1]   # worlds/detection_arena/excavation<seed>.world (needs make_detection_arena.py --arena)

Excavator (base at SITE['excavator']['pos']): the house (cab and counterweight) swings about the base; the arm (boom,
stick and bucket as one rigid 7.5 m link, foot 1.2 m ahead of the swing axis at 2.2 m) pitches. Cycle (~18 s): dig low
at swing heading dig_deg, curl, raise, swing to dump_deg (over the truck), dump, swing back while lowering.
Crane (mast at SITE['crane']['pos'], 10 m): the load (1.2 x 1.2 x 0.8 m box on a cable) hangs 6 m out and slews between
pick_deg and set_deg, lifted to carry height in between (~32 s cycle).
"""
import argparse
import os

import numpy as np

from make_detection_arena import WORLDS, box, capsule, mat

SITE = {
    'excavator': dict(pos=(19.0, 4.0), dig_deg=180.0, dump_deg=270.0, foot=(1.2, 2.2), arm_len=7.5, dig_elev=-16.0, carry_elev=20.0),
    'crane': dict(pos=(4.0, -8.0), radius=6.0, pick_deg=0.0, set_deg=110.0, mast_h=10.0, low=0.3, carry=1.2, slew_rate=0.25),
}
TRUCK = (19.0, -3.0)   # under the excavator's dump heading
BERMS = [((6.0, 7.0), (1.5, 8.0)), ((-3.0, -5.0), (4.0, 1.5)), ((24.0, -7.0), (1.5, 8.0))]  # spoil berms (centre, size), 2.6 m: hide what is behind
PEOPLE = ['red', 'yellow', 'blue']


def excavator_pose(t):
    """(swing yaw, arm elevation) in rad at time t: a dig-swing-dump-swing cycle of smoothstepped keyframes."""
    e = SITE['excavator']
    d, u = np.radians(e['dig_deg']), np.radians(e['dump_deg'])
    lo, hi = np.radians(e['dig_elev']), np.radians(e['carry_elev'])
    # (duration, yaw at end, elevation at end): start of cycle at the dig, arm low
    keys = [(3.0, d, lo + np.radians(8)),  # curl: drag the bucket in, rising a little
            (2.0, d, hi),                     # raise
            (4.0, u, hi),                     # swing to the truck
            (2.0, u, hi + np.radians(6)),     # dump
            (4.5, d, lo + np.radians(10)),    # swing back, lowering
            (2.5, d, lo)]                     # lower into the cut
    period = sum(k[0] for k in keys)
    s = t % period
    y0, a0 = d, lo
    for dur, y1, a1 in keys:
        if s <= dur:
            f = s / dur; f = f * f * (3 - 2 * f)
            return y0 + f * (y1 - y0), a0 + f * (a1 - a0)
        s -= dur; y0, a0 = y1, a1
    return d, lo


def crane_pose(t):
    """(slew angle rad, load bottom height m) at time t: pick low, lift, slew, lower, set, lift, slew back, lower."""
    c = SITE['crane']
    p, q = np.radians(c['pick_deg']), np.radians(c['set_deg'])
    slew = abs(q - p) / c['slew_rate']
    keys = [(3.0, p, c['low']), (3.0, p, c['carry']), (slew, q, c['carry']), (3.0, q, c['low']),
            (3.0, q, c['low']), (3.0, q, c['carry']), (slew, p, c['carry']), (3.0, p, c['low'])]
    period = sum(k[0] for k in keys)
    s = t % period
    a0, h0 = p, c['low']
    for dur, a1, h1 in keys:
        if s <= dur:
            f = s / dur; f = f * f * (3 - 2 * f)
            return a0 + f * (a1 - a0), h0 + f * (h1 - h0)
        s -= dur; a0, h0 = a1, h1
    return p, c['low']


def hazard_capsules(t):
    """True shapes of the moving machines at time t, as capsules (name, a xyz, b xyz, radius): for scoring clearance."""
    e, c = SITE['excavator'], SITE['crane']
    yaw, el = excavator_pose(t)
    E = np.array([*e['pos'], 0.0])
    u = np.array([np.cos(yaw), np.sin(yaw), 0.0])
    foot = E + u * e['foot'][0] + np.array([0, 0, e['foot'][1]])
    arm = np.array([np.cos(el) * u[0], np.cos(el) * u[1], np.sin(el)])
    tip = foot + arm * e['arm_len']
    house = E + np.array([0, 0, 1.6]) - u * 0.4
    sl, h = crane_pose(t)
    C = np.array([*c['pos'], 0.0])
    load = C + c['radius'] * np.array([np.cos(sl), np.sin(sl), 0.0])
    out = [('excavator_arm', foot, tip, 0.35), ('excavator_bucket', tip, tip, 0.7),
           ('excavator_house', house - u * 0.8, house + u * 0.8, 1.35),
           ('crane_load', load + [0, 0, h + 0.4], load + [0, 0, h + 0.4], 0.85),
           ('crane_cable', load + [0, 0, h + 0.8], load + [0, 0, c['mast_h'] - 0.5], 0.03)]
    return [(n, np.asarray(a, float), np.asarray(b, float), r) for n, a, b, r in out]


def _rz(a):
    return np.array([[np.cos(a), -np.sin(a), 0], [np.sin(a), np.cos(a), 0], [0, 0, 1]])


def hazard_boxes(t):
    """Exact shapes of the moving machines at time t, for scoring clearance: oriented boxes (name, centre, R, half
    extents) as in the world file, and the crane cable as a thin segment (name, a, b, radius)."""
    e, c = SITE['excavator'], SITE['crane']
    yaw, el = excavator_pose(t)
    E = np.array([*e['pos'], 0.0]); Rh = _rz(yaw)
    boxes = [('excavator_house', E + Rh @ [-0.4, 0, 1.55], Rh, np.array([1.6, 1.3, 0.55])),
             ('excavator_cab', E + Rh @ [0.7, 0.75, 2.65], Rh, np.array([0.6, 0.5, 0.55]))]
    foot = E + Rh @ [e['foot'][0], 0, 0] + [0, 0, e['foot'][1]]
    Ra = Rh @ np.array([[np.cos(el), 0, -np.sin(el)], [0, 1, 0], [np.sin(el), 0, np.cos(el)]])  # x along the arm, raised by el
    L = e['arm_len']
    boxes += [('excavator_boom', foot + Ra @ [(L - 0.6) / 2, 0, 0], Ra, np.array([(L - 0.6) / 2, 0.25, 0.3])),
              ('excavator_bucket', foot + Ra @ [L - 0.3, 0, -0.2], Ra, np.array([0.5, 0.55, 0.45]))]
    sl, h = crane_pose(t)
    load = np.array([*c['pos'], 0.0]) + c['radius'] * np.array([np.cos(sl), np.sin(sl), 0.0])
    boxes.append(('crane_load', load + [0, 0, h + 0.4], _rz(sl), np.array([0.6, 0.6, 0.4])))
    cable = ('crane_cable', load + [0, 0, h + 0.8], load + [0, 0, c['mast_h'] - 0.5], 0.03)
    return boxes, [cable]


def in_hazard_area(p, pad=1.2):
    """Inside the excavator's swing sector or the crane's slewing arc (padded): where people do not walk."""
    e, c = SITE['excavator'], SITE['crane']
    v = np.asarray(p) - e['pos']; r = np.hypot(*v); a = np.degrees(np.arctan2(v[1], v[0])) % 360
    reach = e['foot'][0] + e['arm_len'] + 0.8
    if r < 2.5 + pad or (r < reach + pad and e['dig_deg'] - 15 <= a <= e['dump_deg'] + 15): return True
    v = np.asarray(p) - c['pos']; r = np.hypot(*v); a = np.degrees(np.arctan2(v[1], v[0])) % 360
    if r < 1.0 + pad or (abs(r - c['radius']) < 1.0 + pad and c['pick_deg'] - 10 <= a <= c['set_deg'] + 10): return True
    if any(np.all(np.abs(np.asarray(p) - c) < np.asarray(sz) / 2 + pad) for c, sz in BERMS): return True
    return np.hypot(*(np.asarray(p) - TRUCK)) < 3.5 + pad


def safe_route(rng, start, n=30, x=(-4.0, 26.0), y=(-10.0, 10.0)):
    pts, p, h = [np.array(start, float)], np.array(start, float), rng.uniform(0, 2 * np.pi)
    while len(pts) < n:
        for _ in range(200):
            hh = h + rng.uniform(-np.radians(110), np.radians(110))
            q = p + rng.uniform(3.0, 8.0) * np.array([np.cos(hh), np.sin(hh)])
            if not (x[0] <= q[0] <= x[1] and y[0] <= q[1] <= y[1]): continue
            if any(in_hazard_area(p + f * (q - p)) for f in np.linspace(0, 1, 20)): continue
            p, h = q, hh; pts.append(q); break
        else:
            h += np.pi
    return pts + [pts[0]]


def make_site(seed):
    rng = np.random.default_rng(seed)
    e, c = SITE['excavator'], SITE['crane']
    models = []
    # excavator: tracked undercarriage (static), house and arm (kinematic, posed by site_motion.py)
    ex, ey = e['pos']
    models.append('<model name="excavator_base"><static>true</static><pose>%.2f %.2f 0 0 0 0</pose><link name="link">%s</link></model>'
                  % (ex, ey, box('tracks', (4.2, 3.2, 0.95), (0, 0, 0.475, 0, 0, 0), 'Corridor/Wall0')))
    house = (box('house', (3.2, 2.6, 1.1), (-0.4, 0, 1.55, 0, 0, 0), 'Corridor/Wall2')
             + box('cab', (1.2, 1.0, 1.1), (0.7, 0.75, 2.65, 0, 0, 0), 'Corridor/Plywood'))
    models.append('<model name="excavator_house"><pose>%.2f %.2f 0 0 0 %.3f</pose><link name="link"><gravity>false</gravity><kinematic>true</kinematic>%s</link></model>'
                  % (ex, ey, np.radians(e['dig_deg']), house))
    L = e['arm_len']
    arm = (box('boom', (L - 0.6, 0.5, 0.6), ((L - 0.6) / 2, 0, 0, 0, 0, 0), 'Corridor/Painting05')
           + box('bucket', (1.0, 1.1, 0.9), (L - 0.3, 0, -0.2, 0, 0, 0), 'Corridor/Wall0'))
    models.append('<model name="excavator_arm"><pose>%.2f %.2f %.2f 0 0 0</pose><link name="link"><gravity>false</gravity><kinematic>true</kinematic>%s</link></model>'
                  % (ex - e['foot'][0], ey, e['foot'][1], arm))
    # crane: mast (static), load and cable (kinematic)
    cx, cy = c['pos']
    models.append('<model name="crane_mast"><static>true</static><pose>%.2f %.2f 0 0 0 0</pose><link name="link">%s%s</link></model>'
                  % (cx, cy, box('mast', (0.8, 0.8, c['mast_h']), (0, 0, c['mast_h'] / 2, 0, 0, 0), 'Corridor/Wall0'),
                     box('pad', (3.0, 3.0, 0.4), (0, 0, 0.2, 0, 0, 0), 'Corridor/Wall1')))
    cable_len = c['mast_h'] - 0.5
    load = (box('load', (1.2, 1.2, 0.8), (0, 0, 0.4, 0, 0, 0), 'Corridor/Plywood')
            + '<visual name="cable_v"><pose>0 0 %.3f 0 0 0</pose><geometry><cylinder><radius>0.03</radius><length>%.3f</length></cylinder></geometry></visual>'
              '<collision name="cable_c"><pose>0 0 %.3f 0 0 0</pose><geometry><cylinder><radius>0.03</radius><length>%.3f</length></cylinder></geometry></collision>'
            % (0.8 + cable_len / 2, cable_len, 0.8 + cable_len / 2, cable_len))
    models.append('<model name="crane_load"><pose>%.2f %.2f %.2f 0 0 0</pose><link name="link"><gravity>false</gravity><kinematic>true</kinematic>%s</link></model>'
                  % (cx + c['radius'], cy, c['low'], load))
    # static clutter: the dump truck under the dump heading, the cut's spoil pile, material stacks clear of everything
    models.append('<model name="truck"><static>true</static><pose>%.2f %.2f 0 0 0 0</pose><link name="link">%s%s</link></model>'
                  % (*TRUCK, box('bed', (2.6, 5.0, 1.6), (0, 0.5, 1.3, 0, 0, 0), 'Corridor/Wall3'), box('cabin', (2.4, 1.6, 2.4), (0, -2.9, 1.2, 0, 0, 0), 'Corridor/Wall5')))
    dig = np.array(e['pos']) + (e['foot'][0] + L * np.cos(np.radians(e['dig_elev'])) + 1.6) * np.array([np.cos(np.radians(e['dig_deg'])), np.sin(np.radians(e['dig_deg']))])
    models.append('<model name="spoil"><static>true</static><pose>%.2f %.2f 0 0 0 0.4</pose><link name="link">%s</link></model>'
                  % (dig[0], dig[1] + 2.5, box('pile', (2.5, 2.0, 0.9), (0, 0, 0.45, 0, 0, 0), 'Corridor/Floor1')))
    for k, (c, sz) in enumerate(BERMS):
        models.append('<model name="berm_%d"><static>true</static><pose>%.2f %.2f 0 0 0 0</pose><link name="link">%s</link></model>'
                      % (k, c[0], c[1], box('b', (*sz, 2.6), (0, 0, 1.3, 0, 0, 0), 'Corridor/Floor%d' % (k % 3))))
    placed = []
    while len(placed) < 5:
        q = np.array([rng.uniform(-3, 25), rng.uniform(-10, 10)])
        if in_hazard_area(q, pad=1.5) or np.hypot(*q) < 4 or any(np.hypot(*(q - p)) < 4 for p in placed): continue
        placed.append(q)
        w, d, h = rng.uniform(0.8, 2.0), rng.uniform(0.8, 2.0), rng.uniform(0.8, 2.4)
        models.append('<model name="stack_%d"><static>true</static><pose>%.2f %.2f 0 0 0 %.2f</pose><link name="link">%s</link></model>'
                      % (len(placed), q[0], q[1], rng.uniform(0, np.pi), box('s', (w, d, h), (0, 0, h / 2, 0, 0, 0), 'Corridor/Plywood')))
    # people: capsules on loops that keep out of the machines' reach
    for k, color in enumerate(PEOPLE):
        while True:
            start = np.array([rng.uniform(-3, 25), rng.uniform(-9, 9)])
            if not in_hazard_area(start) and np.hypot(*start) > 4 and all(np.hypot(*(start - p)) > 1.5 for p in placed): break
        pts = safe_route(rng, start)
        wps = ''.join('<waypoint>%.3f %.3f 0</waypoint>' % (q[0], q[1]) for q in pts)
        models.append('<model name="person_%s"><pose>%.3f %.3f 0 0 0 0</pose><link name="link"><gravity>false</gravity>%s</link>'
                      '<plugin name="walk_%s" filename="libobstaclePathPlugin.so"><loop>1</loop><velocity>%.3f</velocity>'
                      '<angular_velocity>2.0</angular_velocity><path>%s</path></plugin></model>'
                      % (color, pts[0][0], pts[0][1], capsule(color), color, rng.uniform(0.6, 1.0), wps))
    world = ('<?xml version="1.0"?>\n<!-- excavation site (seed %d): generated by make_excavation_site.py; machines driven by site_motion.py -->\n'
             '<sdf version="1.6"><world name="excavation%d">\n'
             '<include><uri>model://sun</uri></include><include><uri>model://ground_plane</uri></include>\n'
             '<include><uri>model://detection_arena</uri></include>\n%s\n</world></sdf>\n' % (seed, seed, '\n'.join(models)))
    os.makedirs(WORLDS, exist_ok=True)
    out = os.path.join(WORLDS, 'excavation%d.world' % seed)
    open(out, 'w').write(world)
    return out


if __name__ == '__main__':
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--seed', type=int, default=1)
    print('Wrote', make_site(ap.parse_args().seed))
