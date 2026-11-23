"""Worlds and drones for the Simulator galleries, read straight from uav_simulator (no ROS needed).

worlds(): every worlds/**/*.world, grouped by folder, each with its collision shapes in world coordinates (for the
  preview thumbnails) and where the drone starts. The start comes from config.json (sim.spawn, by file or folder), else
  from a "drone start (x, y, z)" note in the world file (make_excavation_pit.py writes one), else (0, 0, 0.1).
drones(): every urdf/*.urdf with its sensors (from the Gazebo <sensor> blocks: kind, pose) and its body size, so the
  gallery and the 3D view can draw it until real CAD models are added. xacro expressions are evaluated with the
  file's xacro:arg defaults.
Results are cached by file modification time.
"""
import glob
import math
import os
import re
import xml.etree.ElementTree as ET

import numpy as np

_cache = {}
STL_RECORD = np.dtype([('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')])   # binary STL: normal, 3 vertices, attribute


def cached(path, fn):
    key = (path, os.path.getmtime(path))
    if key not in _cache:
        _cache[key] = fn(path)
    return _cache[key]


def pose_matrix(text, xyz=None, rpy=None):
    """4x4 from an SDF '<pose>x y z r p y</pose>' (or URDF xyz/rpy strings)."""
    if text is not None:
        v = [float(x) for x in text.split()] + [0.0] * 6
        x, y, z, r, p, w = v[:6]
    else:
        x, y, z = [float(a) for a in (xyz or '0 0 0').split()]
        r, p, w = [float(a) for a in (rpy or '0 0 0').split()]
    cr, sr, cp, sp, cw, sw = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(w), math.sin(w)
    T = np.eye(4)
    T[:3, :3] = [[cw * cp, cw * sp * sr - sw * cr, cw * sp * cr + sw * sr],
                 [sw * cp, sw * sp * sr + cw * cr, sw * sp * cr - cw * sr],
                 [-sp, cp * sr, cp * cr]]
    T[:3, 3] = x, y, z
    return T


def child_pose(el):
    p = el.find('pose')
    return pose_matrix(p.text if p is not None and p.text else '0 0 0 0 0 0')


# ------------------------------------------------------------------ worlds
MODEL_PATHS = []   # set by worlds(): uav_simulator/models, then GAZEBO_MODEL_PATH, then ~/.gazebo/models


def model_sdf(name):
    """The SDF file of model://name (its model.config names it; model.sdf otherwise)."""
    for base in MODEL_PATHS:
        d = os.path.join(base, name)
        if not os.path.isdir(d):
            continue
        cfg = os.path.join(d, 'model.config')
        if os.path.isfile(cfg):
            try:
                sdfs = [e.text.strip() for e in ET.parse(cfg).getroot().findall('sdf') if e.text]
            except ET.ParseError:
                sdfs = []
            for f in sdfs + ['model.sdf']:
                if os.path.isfile(os.path.join(d, f)):
                    return os.path.join(d, f)
        if os.path.isfile(os.path.join(d, 'model.sdf')):
            return os.path.join(d, 'model.sdf')
    return None
START_NOTE = re.compile(r'drone start \(\s*([-\d.]+),\s*([-\d.]+),\s*([-\d.]+)\s*\)')


def parse_world(path, max_items=3000):
    text = open(path, errors='replace').read()
    start = START_NOTE.search(text)
    items, dynamic = [], 0
    try:
        root = ET.fromstring(text)
    except ET.ParseError:
        return {'items': [], 'dynamic': 0, 'start': None, 'error': 'could not parse'}
    world = root.find('world') if root.tag == 'sdf' else root

    def walk_model(model, T, depth=0):
        nonlocal dynamic
        T = T @ child_pose(model)
        static = (model.findtext('static') or '').strip() in ('1', 'true')
        moving = not static and (model.find('plugin') is not None or model.find('.//plugin') is not None)
        if moving:
            dynamic += 1
        for link in model.findall('link'):
            TL = T @ child_pose(link)
            for col in link.findall('collision'):
                g = col.find('geometry')
                if g is None or len(items) >= max_items:
                    continue
                M = TL @ child_pose(col)
                m = [round(float(a), 4) for a in M.T.ravel()]          # column-major, as three.js Matrix4.fromArray
                box, cyl, sph = g.find('box'), g.find('cylinder'), g.find('sphere')
                if box is not None:
                    s = [float(a) for a in box.findtext('size', '1 1 1').split()]
                    items.append({'t': 'box', 'm': m, 's': s, 'd': int(moving)})
                elif cyl is not None:
                    items.append({'t': 'cyl', 'm': m, 's': [float(cyl.findtext('radius', '0.5')), float(cyl.findtext('length', '1'))], 'd': int(moving)})
                elif sph is not None:
                    items.append({'t': 'sph', 'm': m, 's': [float(sph.findtext('radius', '0.5'))], 'd': int(moving)})
        for sub in model.findall('model'):
            walk_model(sub, T, depth)
        for inc in model.findall('include'):
            walk_include(inc, T, depth)

    def walk_include(inc, T, depth=0):
        uri = (inc.findtext('uri') or '').strip()
        if not uri.startswith('model://') or depth > 4:
            return
        sdf = model_sdf(uri[len('model://'):].split('/')[0])
        if sdf is None:
            return
        try:
            r = ET.parse(sdf).getroot()
        except ET.ParseError:
            return
        T = T @ child_pose(inc)
        for model in r.findall('model'):
            model = model if inc.find('static') is None else model
            walk_model(model, T, depth + 1)

    def walk_children(el, T, depth):
        for inc in el.findall('include'):
            walk_include(inc, T, depth)

    if world is not None:
        for model in world.findall('model'):
            walk_model(model, np.eye(4))
        walk_children(world, np.eye(4), 0)
        for actor in world.findall('actor'):
            dynamic += 1
    return {'items': items, 'dynamic': dynamic,
            'start': [float(start.group(i)) for i in (1, 2, 3)] if start else None}


def worlds(root, spawn_cfg):
    MODEL_PATHS[:] = [os.path.join(os.path.dirname(os.path.abspath(root)), 'models')] + \
        [p for p in os.environ.get('GAZEBO_MODEL_PATH', '').split(':') if p] + [os.path.expanduser('~/.gazebo/models')]
    out = []
    for path in sorted(glob.glob(os.path.join(root, '**', '*.world'), recursive=True)):
        rel = os.path.relpath(path, root)
        info = cached(path, parse_world)
        folder = rel.split(os.sep)[0] if os.sep in rel else ''
        start = spawn_cfg.get(rel) or spawn_cfg.get(folder) or info['start'] or [0.0, 0.0, 0.1]
        lo, hi = np.full(3, np.inf), np.full(3, -np.inf)
        for it in info['items']:
            M = np.array(it['m']).reshape(4, 4).T
            half = (np.array(it['s']) / 2 if it['t'] == 'box' else
                    np.array([it['s'][0], it['s'][0], it['s'][1] / 2]) if it['t'] == 'cyl' else np.full(3, it['s'][0]))
            r = np.abs(M[:3, :3]) @ half                       # axis-aligned half extent of the rotated shape
            lo, hi = np.minimum(lo, M[:3, 3] - r), np.maximum(hi, M[:3, 3] + r)
        ext = (hi - lo).round(1).tolist() if info['items'] else [0, 0, 0]
        kinds = {}
        for it in info['items']:
            M = np.array(it['m']).reshape(4, 4).T
            half = (np.array(it['s']) / 2 if it['t'] == 'box' else
                    np.array([it['s'][0], it['s'][0], it['s'][1] / 2]) if it['t'] == 'cyl' else np.full(3, it['s'][0]))
            e = 2 * np.abs(M[:3, :3]) @ half                    # axis-aligned size of the shape
            h, lo_, hi_ = e[2], min(e[0], e[1]), max(e[0], e[1])
            if it['d']: k = 'moving loads' if min(e) >= 0.9 else 'people' if hi_ < 1.2 and h < 2.2 else 'moving machines'
            elif h < 0.4 and hi_ > 2: k = 'decks and floors'
            elif hi_ > 3 and lo_ < 1.2 and h > 1.5: k = 'walls'
            elif h >= 1.5 and hi_ < 0.8: k = 'columns'
            elif h < 0.8 and hi_ > 2 and lo_ < 0.8: k = 'beams and struts'
            else: k = 'blocks'
            kinds[k] = kinds.get(k, 0) + 1
        out.append({'file': rel, 'group': folder or 'other', 'obstacles': kinds, 'name': os.path.splitext(os.path.basename(rel))[0],
                    'items': info['items'], 'dynamic': info['dynamic'], 'start': start, 'extent': ext})
    return out


# ------------------------------------------------------------------ drones
XARG = re.compile(r'<xacro:arg\s+name="([^"]+)"\s+default="([^"]*)"')
XPROP = re.compile(r'<xacro:property\s+name="([^"]+)"\s+value="([^"]*)"')


def xacro_eval(text):
    """Enough xacro for our URDFs: $(arg x) from defaults, then ${expr} with radians/pi and the numeric xacro:property values."""
    defaults = dict(XARG.findall(text))
    text = re.sub(r'\$\(arg\s+([^)]+)\)', lambda m: defaults.get(m.group(1).strip(), '0'), text)
    env = {'radians': math.radians, 'degrees': math.degrees, 'pi': math.pi, 'cos': math.cos, 'sin': math.sin, 'float': float}
    for k, v in XPROP.findall(text):
        try:
            env[k] = float(v)
        except ValueError:
            pass
    return re.sub(r'\$\{([^}]*)\}', lambda m: repr(eval(m.group(1), {'__builtins__': {}}, env)), text)


SENSOR_KIND = {'ray': 'lidar', 'gpu_ray': 'lidar', 'depth': 'depth camera', 'camera': 'camera',
               'wideanglecamera': 'fisheye camera', 'imu': 'imu'}


def parse_drone(path):
    raw = open(path, errors='replace').read()
    text = xacro_eval(raw)
    root = ET.fromstring(text)
    sensors = []
    for gz in root.findall('gazebo'):
        for s in gz.findall('sensor'):
            kind = SENSOR_KIND.get(s.get('type'), s.get('type'))
            if 'ideal' in (s.get('name') or '') or 'omni360' in (s.get('name') or ''):
                kind = '360 depth (ideal)'
            if kind == 'imu':
                continue
            p = s.findtext('pose') or '0 0 0 0 0 0'
            v = [float(a) for a in p.split()] + [0] * 6
            sensors.append({'name': s.get('name'), 'kind': kind, 'xyz': v[:3], 'rpy': v[3:6]})
    scale, span = 1.0, 0.5
    mesh = root.find('.//visual/geometry/mesh')
    if mesh is not None and mesh.get('scale'):
        scale = float(mesh.get('scale').split()[0])
        span = 0.34 * scale                                   # CERLAB_quadcopter.stl spans 0.34 m at scale 1
    out = {'sensors': sensors, 'span': round(span, 3)}
    m = re.search(r'package://uav_simulator/urdf/([^/"]+)/meshes/', text)          # built from CAD: meshes + model.json
    model_json = m and os.path.join(os.path.dirname(path), m.group(1), 'model.json')
    if model_json and os.path.isfile(model_json):
        import json
        info = json.load(open(model_json))
        out['model'] = {'dir': m.group(1), 'groups': info['groups'], 'hinge_xyz': info['hinge_xyz'],
                        'lidar_from_hinge': info['lidar_from_hinge'], 'bounds': info['bounds'],
                        'components': info.get('components', [])}
        out['span'] = round(info['bounds'][1][0] - info['bounds'][0][0], 3)
        out['sensors'] = [s for s in sensors if s['kind'] != 'lidar']      # the LiDAR is drawn from the hinge
    if not out.get('model') and mesh is not None:
        # a drone drawn from one mesh (the original CERLAB quadcopter): its STL (the .dae Gazebo draws has an STL twin)
        f = re.sub(r'^package://uav_simulator/urdf/', '', mesh.get('filename', ''))
        stl = os.path.splitext(f)[0] + '.stl'
        if os.path.isfile(os.path.join(os.path.dirname(path), stl)):
            vis = root.find('.//visual')
            o = vis.find('origin') if vis is not None else None
            xyz = [float(a) for a in (o.get('xyz', '0 0 0') if o is not None else '0 0 0').split()]
            lo, hi = stl_bounds(os.path.join(os.path.dirname(path), stl))
            baked = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'web', 'meshes', os.path.basename(stl))
            url = '/meshes/' + os.path.basename(stl) if os.path.isfile(baked) else '/models/' + stl   # tools/bake_mesh_colors.py
            out['mesh'] = {'url': url, 'scale': scale, 'xyz': xyz,
                           'bounds': [[round(lo[i] * scale + xyz[i], 4) for i in range(3)], [round(hi[i] * scale + xyz[i], 4) for i in range(3)]]}
    out['xarg_defaults'] = {}                                  # the URDF's numeric xacro args (drones() keeps the tunable ones)
    for k, v in XARG.findall(raw):
        try:
            out['xarg_defaults'][k] = float(v)
        except ValueError:
            pass
    if out.get('model') and 'insta360_x' in raw and 'insta360_x4' in raw:
        # the 360 camera (make_cad_urdf.py): not in model.json; the omni_link visual, origin at the lens centre, level,
        # placed by the xacro args insta360_x / insta360_z / insta360_sideways (lenses left and right: on the drone's
        # axis, leaned forward the URDF's insta360_side_tilt_deg)
        x, z = out['xarg_defaults'].get('insta360_x', 0.0), out['xarg_defaults'].get('insta360_z', 0.0)
        out['cam_side_tilt'] = float(dict(XPROP.findall(raw)).get('insta360_side_tilt_deg', 0))
        mesh = os.path.join(os.path.dirname(path), out['model']['dir'], 'meshes', 'insta360_x4.stl')
        out['extra_parts'] = [{'name': 'insta360_x4', 'mesh': 'insta360_x4', 'xyz': [x, 0.0, z], 'edit': 'mount',
                               'bounds': stl_bounds(mesh)}]
        out['model']['components'] = out['model']['components'] + [
            {'label': '360 camera', 'detail': 'Insta360 X4 · /insta360/front + /back',
             'xyz': [x, 0.0, z], 'edit': 'mount', 'part': 'insta360_x4'}]
    return out


def stl_bounds(path):
    """[[min], [max]] of a binary STL's vertices."""
    raw = open(path, 'rb').read()
    n = int.from_bytes(raw[80:84], 'little')
    v = np.frombuffer(raw[84:84 + 50 * n], STL_RECORD)['v'].reshape(-1, 3)
    return [[round(float(a), 4) for a in v.min(0)], [round(float(a), 4) for a in v.max(0)]]


def drones(root, names, xargs=None):
    """Drones named in config.json (sim.drones) first, in that order, then the rest alphabetically. xargs: config.json
    sim.xargs, the settings the Simulator may change; each drone gets the ones its URDF declares, as
    {name: {label, min, max, default}}."""
    out = []
    order = {f: i for i, f in enumerate(names)}
    for path in sorted(glob.glob(os.path.join(root, '*.urdf')), key=lambda p: (order.get(os.path.basename(p), len(order)), p)):
        f = os.path.basename(path)
        meta = names.get(f, {})
        if meta.get('hidden'):
            continue
        try:
            info = cached(path, parse_drone)
        except Exception as e:  # a URDF this reader cannot follow still shows, without sensors
            info = {'sensors': [], 'span': 0.5, 'error': str(e)}
        tunable = {k: dict(spec, default=info['xarg_defaults'][k]) for k, spec in (xargs or {}).items()
                   if not k.startswith('_') and k in info.get('xarg_defaults', {})}
        out.append(dict({k: v for k, v in info.items() if k != 'xarg_defaults'}, xargs=tunable,
                        file=f, label=meta.get('label') or f[:-5].replace('_', ' '), note=meta.get('note', '')))
    return out
