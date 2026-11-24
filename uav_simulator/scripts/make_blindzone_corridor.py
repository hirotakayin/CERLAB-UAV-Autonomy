#!/usr/bin/env python3
"""A long, photo-textured corridor and scripted encounters for testing camera-only obstacle sensing in the lidar's blind
zone (the tilted Mid-360 sees nothing behind the drone beyond about +-110 deg, or low around it): time to contact and
speed from optic flow, against Gazebo ground truth. No planner: the drone flies a fixed line (MALT weave_flight.py with
amplitude 0) or hovers, so detection is isolated from avoidance.

  models/blindzone_corridor/     corridor x [X0, X1], y [-3, 3], walls 4 m, open top: walls in 5 m sections of
                                 board-formed concrete, brick, stone and plain plaster (low texture: the hard case for flow),
                                 lighting baked in (make_excavation_slab_textured.py), static clutter along the walls
  worlds/blindzone_corridor/<scenario>.world   the corridor and one target (person, 0.5 x 0.5 x 1.8 m body)
  worlds/blindzone_corridor/scenarios.json     per scenario: the drone's flight and the target's path, for the scorer

  make_blindzone_corridor.py --textures ~/.cache/ambientcg [--seed 5] [--texture high|low]

Scenarios (drone at 1.5 m on y = 0; the encounter is timed to sim time T_MEET; every target approaches from behind,
inside the lidar's blind zone):
  overtake_c1    drone 1 m/s along +x; target overtaking on its line, closing at 1 m/s; stops 0.8 m short (collision course)
  overtake_c2    the same, closing at 2 m/s
  overtake_m1    closing at 1 m/s, 1.0 m to the side: passes the drone (near miss) and walks on ahead
  hover_c        drone hovering at x = 0; person walking at it from behind at 1.2 m/s, stops 0.8 m short
  hover_m        the same, 1.0 m to the side, walks past

The target's path plugin (libobstaclePathPlugin) moves it from sim time 0 and makes each segment last a whole number
of seconds (int(distance / velocity)), so every path here is a whole number of seconds long at its speed.
"""
import argparse
import importlib.util
import json
import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SIM = os.path.normpath(os.path.join(HERE, '..'))
NAME = 'blindzone_corridor'
MODEL = os.path.join(SIM, 'models', NAME)
WORLDS = os.path.join(SIM, 'worlds', NAME)
_spec = importlib.util.spec_from_file_location('slab', os.path.join(HERE, 'make_excavation_slab_textured.py'))
slab = importlib.util.module_from_spec(_spec); _spec.loader.exec_module(slab)

X0, X1, HALF_W, H = -55.0, 45.0, 3.0, 4.0
SECTION = 5.0
T_START, MEET, RAMP = 10.0, 8.0, 1.5     # weave_flight: start (sim s), s into the weave of the encounter, its speed ramp
T_MEET = T_START + MEET                  # 18 s: whole seconds, so path durations are too
HEIGHT, STOP = 1.2, 0.8                  # drone height (camera 1.44 m: a 1.8 m person clearly hits); a collision-course
                                         # target stops this far (centre to centre, horizontally) short
SCENARIOS = {  # name: (drone speed, target speed, lateral offset, collision course)
    'overtake_c1': (1.0, 2.0, 0.0, True),
    'overtake_c2': (1.0, 3.0, 0.0, True),
    'overtake_m1': (1.0, 2.0, 1.0, False),
    'hover_c': (0.0, 1.2, 0.0, True),
    'hover_m': (0.0, 1.2, 1.0, False),
}
WALLS = {  # look: (texture sets, stain, panels, tie holes)
    'concrete': (('Concrete034', 'Concrete046'), 'Concrete036', (2.4, 1.2), True),
    'brick': (('Bricks105', 'Bricks105'), 'Concrete036', None, False),
    'stone': (('Bricks109A', 'Bricks109A'), 'Concrete036', None, False),
    'plaster': (('PaintedPlaster017', 'PaintedPlaster017'), 'Concrete036', None, False),
}
MIX = {'high': {'concrete': 0.35, 'brick': 0.25, 'stone': 0.2, 'plaster': 0.2},
       'low': {'concrete': 0.15, 'brick': 0.05, 'stone': 0.0, 'plaster': 0.8}}


def floor_texture(S, rng, w_m, h_m, ppm):
    h, w = int(h_m * ppm), int(w_m * ppm)
    img = slab.mix(S['Concrete031'].canvas(rng, h, w, ppm), S['Concrete042A'].canvas(rng, h, w, ppm), slab.soft_mask(rng, h, w, 0.5, sharp=8, base=4))
    img = slab.mix(img, S['Ground054'].canvas(rng, h, w, ppm) * 0.9, slab.soft_mask(rng, h, w, 0.2, sharp=10, base=6) * 0.7)
    img = slab.tone(img, rng, 0.10)
    yy = np.mgrid[0:h, 0:w][0] / ppm
    img *= (1 - 0.3 * np.exp(-np.minimum(yy, h_m - yy) / 0.5))[..., None]   # occlusion along both walls
    return np.clip(img, 0, 1)


def drone_x(speed, t):
    """weave_flight.py's x (from home) t s into the weave."""
    return speed * (t - RAMP / 2 if t > RAMP else t * t / (2 * RAMP))


def target_path(speed, v_t, offset, collision):
    """(waypoints, meta) for the target: on the drone's line (offset 0) it stops STOP short of the drone at T_MEET; to the
    side it is level with the drone at T_MEET and walks on for 10 s."""
    x_meet = drone_x(speed, MEET)                       # the drone's x at T_MEET (home x = 0)
    if collision:
        end = np.array([x_meet - STOP, offset]); dur = T_MEET
        start = end - np.array([v_t * dur, 0.0])
    else:
        meet = np.array([x_meet, offset]); start = meet - np.array([v_t * T_MEET, 0.0])
        dur = T_MEET + 10.0; end = start + np.array([v_t * dur, 0.0])
    if start[0] < X0 + 1.0: raise ValueError(f'target would start at x {start[0]:.1f}, behind the corridor (X0 {X0})')
    return [start, end], dict(start=start.tolist(), end=end.tolist(), speed=v_t, t_move=[0.0, dur])


def target_model(path, v_t):
    """Gazebo's PoseAnimation splines between keyframes: with only the two ends the target eases in and out (0.6x to 1.25x
    its speed). A keyframe every second along the line keeps the speed constant except in the first and last second."""
    a, b = path
    visual = '<visual name="visual"><pose>0 0 -0.02 0 0 1.57</pose><geometry><mesh><uri>model://person/meshes/walking.dae</uri></mesh></geometry></visual>'
    body = '<collision name="body"><pose>0 0 0.9 0 0 0</pose><geometry><box><size>0.5 0.5 1.8</size></box></geometry></collision>'
    # each segment 1 s long (+ 1e-6: the plugin truncates distance / velocity to whole seconds, 0.999.. must not become 0)
    n = int(round(np.linalg.norm(b - a) / v_t))
    step = (b - a) / n * (1 + 1e-6)
    wps = ''.join('<waypoint>%.6f %.6f 0</waypoint>' % tuple(a + k * step) for k in range(n + 1))
    return ('<model name="target"><pose>%.4f %.4f 0 0 0 0</pose><link name="link"><gravity>false</gravity>%s%s</link>'
            '<plugin name="obstacle_motion" filename="libobstaclePathPlugin.so"><loop>0</loop><orientation>false</orientation>'
            '<velocity>%.4f</velocity><path>%s</path></plugin></model>' % (a[0], a[1], visual, body, v_t, wps))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--textures', required=True, help='folder of ambientCG 1K-JPG sets (see make_excavation_slab_textured.py), '
                    'also Bricks105 Bricks109A PaintedPlaster017')
    ap.add_argument('--seed', type=int, default=5)
    ap.add_argument('--texture', choices=list(MIX), default='high', help='wall mix: high (mostly textured) or low (mostly plain plaster)')
    ap.add_argument('--ppm', type=int, default=100)
    ap.add_argument('--clutter', type=float, default=1.0)
    a = ap.parse_args()
    rng = np.random.default_rng(a.seed)
    tiles = {'Concrete046': 2.5, 'Concrete034': 2.5, 'Concrete036': 2.0, 'Concrete031': 2.0, 'Concrete042A': 2.0,
             'Ground054': 2.0, 'Bricks105': 1.4, 'Bricks109A': 1.6, 'PaintedPlaster017': 2.0}
    S = {k: slab.TexSet(os.path.expanduser(a.textures), k, m) for k, m in tiles.items()}
    mesh_dir = os.path.join(MODEL, 'meshes'); os.makedirs(mesh_dir, exist_ok=True)
    for f in os.listdir(mesh_dir): os.remove(os.path.join(mesh_dir, f))
    X, Y, Z = np.eye(3)
    looks, p = list(MIX[a.texture]), np.array(list(MIX[a.texture].values()))
    surfaces, used = [], {}
    edges = np.arange(X0, X1 + 1e-6, SECTION)
    for k in range(len(edges) - 1):
        xa, xb = edges[k], edges[k + 1]
        # seen from the front, right = up x normal
        for side, (y, eu, p0x, n) in {'s': (-HALF_W, -X, xb, Y), 'n': (HALF_W, X, xa, -Y)}.items():
            look = looks[rng.choice(len(looks), p=p / p.sum())]; used[look] = used.get(look, 0) + 1
            sets, stain, panel, ties = WALLS[look]
            img = slab.wall_texture(S, rng, SECTION, H, a.ppm, sets=sets, stain=stain, panel=panel, ties=ties,
                                    streaks=look != 'plaster' or rng.random() < 0.5, corner_ao=False)
            surfaces.append((f'wall_{side}{k:02d}', img, (p0x, y, 0.0), eu, Z, SECTION, H, n))
        surfaces.append((f'floor{k:02d}', floor_texture(S, rng, SECTION, 2 * HALF_W, a.ppm), (xa, -HALF_W, 0.0), X, Y, SECTION, 2 * HALF_W, Z))
    for name, x, eu, n in (('end_w', X0, Y, X), ('end_e', X1, -Y, -X)):
        y0 = HALF_W if eu[1] < 0 else -HALF_W
        surfaces.append((name, slab.wall_texture(S, rng, 2 * HALF_W, H, a.ppm), (x, y0, 0.0), eu, Z, 2 * HALF_W, H, n))
    parts = []
    T = 0.3                                             # wall thickness (collision and plain visual behind the textured face)
    for name, size, c in (('s', (X1 - X0 + 2 * T, T, H), ((X0 + X1) / 2, -HALF_W - T / 2, H / 2)), ('n', (X1 - X0 + 2 * T, T, H), ((X0 + X1) / 2, HALF_W + T / 2, H / 2)),
                          ('w', (T, 2 * HALF_W, H), (X0 - T / 2, 0, H / 2)), ('e', (T, 2 * HALF_W, H), (X1 + T / 2, 0, H / 2))):
        geo = '<geometry><box><size>%.3f %.3f %.3f</size></box></geometry>' % size
        parts.append(f'<collision name="wall_{name}"><pose>{c[0]:.3f} {c[1]:.3f} {c[2]:.3f} 0 0 0</pose>{geo}</collision>')
        parts.append(f'<visual name="wall_{name}_box"><pose>{c[0]:.3f} {c[1]:.3f} {c[2]:.3f} 0 0 0</pose>{geo}'
                     '<material><ambient>0.5 0.5 0.5 1</ambient><diffuse>0.55 0.55 0.53 1</diffuse></material></visual>')
    for name, img, p0, eu, ev, w, h, n in surfaces:
        slab.save_tex(img, os.path.join(mesh_dir, f'{name}.jpg'))
        slab.quad_obj(os.path.join(mesh_dir, f'{name}.obj'), f'{NAME}_{name}', f'{name}.jpg', p0, eu, ev, w, h, n)
        parts.append(f'<visual name="tex_{name}"><geometry><mesh><uri>model://{NAME}/meshes/{name}.obj</uri></mesh></geometry>'
                     '<cast_shadows>false</cast_shadows></visual>')
    open(os.path.join(MODEL, f'{NAME}.sdf'), 'w').write(
        '<?xml version="1.0"?>\n<!-- generated by uav_simulator/scripts/make_blindzone_corridor.py: rerun it instead of editing -->\n'
        f'<sdf version="1.7"><model name="{NAME}"><static>true</static><link name="link">\n' + '\n'.join(parts) + '\n</link></model></sdf>\n')
    open(os.path.join(MODEL, 'model.config'), 'w').write(
        f'<?xml version="1.0"?>\n<model><name>{NAME}</name><version>1.0</version><sdf version="1.7">{NAME}.sdf</sdf>\n'
        f'<description>Corridor x [{X0:g}, {X1:g}], y [-{HALF_W:g}, {HALF_W:g}], walls {H:g} m, open top; texture mix {a.texture} '
        f'(sections: {used}), seed {a.seed}. Textures from ambientCG (CC0).</description></model>\n')

    # static clutter along the walls (false-alarm bait), clear of the middle 4 m where the drone and targets go
    props = []
    for x in np.arange(X0 + 3, X1 - 3, 3.5):
        if rng.random() > 0.55 * a.clutter or abs(x) < 3: continue
        m = rng.choice(['euro_pallet', 'construction_barrel', 'construction_cone', 'cinder_block', 'drc_practice_2x6'])
        y = rng.choice([-1, 1]) * rng.uniform(2.35, 2.6)
        props.append((m, x + rng.uniform(-1, 1), y, rng.uniform(-np.pi, np.pi)))
    inc = '\n'.join(f'<include><name>{m}_{i}</name><uri>model://{m}</uri><static>true</static><pose>{x:.2f} {y:.2f} 0 0 0 {yaw:.2f}</pose></include>'
                    for i, (m, x, y, yaw) in enumerate(props))
    os.makedirs(WORLDS, exist_ok=True)
    sun = np.array([-0.35, 0.25, -0.9]); sun /= np.linalg.norm(sun)
    meta = dict(t_start=T_START, meet=MEET, t_meet=T_MEET, height=HEIGHT, ramp=RAMP, stop=STOP, texture=a.texture, seed=a.seed,
                corridor=dict(x=[X0, X1], half_width=HALF_W, height=H), scenarios={})
    for name, (speed, v_t, offset, collision) in SCENARIOS.items():
        path, tm = target_path(speed, v_t, offset, collision)
        meta['scenarios'][name] = dict(drone_speed=speed, collision=collision, offset=offset, target=tm,
                                       weave=dict(t_start=T_START, speed=speed, amplitude=0.0, height=HEIGHT, duration=MEET + 12.0))
        world = f'''<?xml version="1.0" ?>
<!-- {name}: generated by uav_simulator/scripts/make_blindzone_corridor.py (texture {a.texture}, seed {a.seed}): rerun it instead of editing -->
<sdf version="1.6">
  <world name="blindzone_{name}">
    <scene><ambient>0.45 0.45 0.48 1</ambient><background>0.62 0.72 0.85 1</background><shadows>false</shadows><grid>false</grid></scene>
    <light name="sun" type="directional"><cast_shadows>false</cast_shadows><pose>0 0 30 0 0 0</pose><diffuse>0.95 0.92 0.85 1</diffuse>
      <specular>0.1 0.1 0.1 1</specular><direction>{sun[0]:.3f} {sun[1]:.3f} {sun[2]:.3f}</direction></light>
    <include><uri>model://ground_plane</uri></include>
    {target_model([np.asarray(q) for q in path], v_t)}
    <include><uri>model://{NAME}</uri><pose>0 0 0 0 0 0</pose></include>
{inc}
  </world>
</sdf>
'''
        open(os.path.join(WORLDS, f'{name}.world'), 'w').write(world)
    json.dump(meta, open(os.path.join(WORLDS, 'scenarios.json'), 'w'), indent=1)
    print(f'Wrote {MODEL} ({len(surfaces)} textured surfaces, walls {used}), {len(SCENARIOS)} scenario worlds and scenarios.json in {WORLDS} '
          f'({len(props)} props)')


if __name__ == '__main__':
    main()
