#!/usr/bin/env python3
"""Simplified drone model from a CAD assembly: groups the part instances (cad_assembly.py's parts.npz) as a
model_spec.json says, moves them into the ROS body frame (x forward, y left, z up, origin base_link), takes the
tilting parts back to zero tilt about the hinge, decimates every group to a small triangle budget, and writes
  <out>/meshes/<group>.stl     one binary STL per group (fixed groups in base_link, moving groups in the hinge frame)
  <out>/model.json             hinge position, lidar_link and camera_link offsets, triangle counts, bounds
so the URDF (xacro, hinge angle = lidar_pitch_deg) and Mission Control draw the same thing at any tilt.

  cad_to_model.py parts.npz uav_simulator/urdf/cerlab_lidar_drone/model_spec.json uav_simulator/urdf/cerlab_lidar_drone

Run with the CAD environment: ~/drone_cad/.venv/bin/python (trimesh, fast-simplification).
"""
import argparse
import json
import os

import fast_simplification
import numpy as np
import trimesh

CAD_TO_ROS = np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]], float)   # rows: x <- Z_cad, y <- X_cad, z <- Y_cad


def rot_y(deg):
    a = np.radians(deg); c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('parts'); ap.add_argument('spec'); ap.add_argument('out')
    a = ap.parse_args()
    spec = json.load(open(a.spec))
    z = np.load(a.parts); table = json.loads(bytes(z['table']).decode())
    origin = np.array(spec['base_link_cad'])
    to_ros = lambda P: (np.asarray(P, float) - origin) @ CAD_TO_ROS.T
    hinge = to_ros(spec['hinge']['point'])
    untilt = rot_y(-spec['hinge']['tilt_in_cad_deg'])          # tilting parts back to level, about the hinge

    # assign each instance to the first group whose rules match
    members = {g['name']: [] for g in spec['groups']}
    for t in table:
        if not t['tris']:
            continue
        V = z[t['key'] + '|V']
        for g in spec['groups']:
            if 'path' in g and not t['path'].startswith(g['path']): continue
            if 'path' not in g and t['path'] and g['name'] != 'frame' and not g.get('any_path'): continue
            if 'names' in g and t['name'] not in g['names']: continue
            if 'size' in g and not np.allclose(np.ptp(V, 0), g['size'], atol=g.get('size_tol', 0.004)): continue
            if 'sizes' in g and not any(np.allclose(np.ptp(V, 0), sz, atol=g.get('size_tol', 0.004)) for sz in g['sizes']): continue
            members[g['name']].append(t['key']); break

    os.makedirs(os.path.join(a.out, 'meshes'), exist_ok=True)
    info = {'name': spec['name'], 'groups': {}, 'hinge_xyz': hinge.round(5).tolist(),
            'hinge_axis': [0, 1, 0], 'cad_tilt_deg': spec['hinge']['tilt_in_cad_deg']}
    allV = []
    for g in spec['groups']:
        keys = members[g['name']]
        if not keys:
            print('group %s: no parts' % g['name']); continue
        if g.get('per_instance'):                                   # e.g. propellers: one mesh per part, origin on its hub
            hubs = [to_ros(h) for h in g['hubs_cad']]
            for i, k in enumerate(keys):
                P = to_ros(z[k + '|V']); c = (P.min(0) + P.max(0)) / 2
                hub = min(hubs, key=lambda h: np.hypot(*(h[:2] - c[:2])))
                pm = trimesh.Trimesh(P - hub, z[k + '|F'], process=True); pm.update_faces(pm.nondegenerate_faces())
                if len(pm.faces) > g['tris']:
                    v2, f2 = fast_simplification.simplify(pm.vertices, pm.faces, target_reduction=1 - g['tris'] / len(pm.faces))
                    q = trimesh.Trimesh(v2, f2, process=True)
                    if np.abs(q.bounds - pm.bounds).max() <= 0.002: pm = q
                name = '%s_%d' % (g['name'], i)
                pm.export(os.path.join(a.out, 'meshes', name + '.stl'))
                info['groups'][name] = {'parts': 1, 'tris': len(pm.faces), 'moving': False, 'bounds': (pm.bounds + hub).round(4).tolist(),
                                        'origin': np.round(hub, 4).tolist(), 'rotor': True}
                allV.append(pm.vertices + hub)
            print('%-14s %3d parts  -> one mesh each on its hub, %d triangles each' % (g['name'], len(keys), len(pm.faces)))
            continue
        # each part separately: its share of the group's budget by surface area; tiny shares become an oriented box,
        # so small parts (feet, pins) keep their place and size instead of collapsing
        meshes = []
        for k in keys:
            P = to_ros(z[k + '|V'])
            if g.get('moving'):
                P = (P - hinge) @ untilt.T                              # hinge frame, zero tilt
            pm = trimesh.Trimesh(P, z[k + '|F'], process=True)          # merges the per-face duplicate vertices
            pm.update_faces(pm.nondegenerate_faces())
            if len(pm.faces): meshes.append(pm)
        area = np.array([pm.area for pm in meshes]); before = sum(len(pm.faces) for pm in meshes)
        out_parts = []
        for pm, ar in zip(meshes, area):
            budget = int(g['tris'] * ar / area.sum())
            if budget < 24 or pm.extents.min() < 0.0015 and budget < 60:
                out_parts.append(pm.bounding_box_oriented.to_mesh()); continue
            # simplify; if the outline moves (thin blades lose their tips), retry with twice the triangles. Only small
            # parts may fall back to their box: a big part (a propeller) must keep its shape
            small = pm.extents.max() < 0.03
            q = pm
            while budget < len(pm.faces):
                v2, f2 = fast_simplification.simplify(pm.vertices, pm.faces, target_reduction=1 - budget / len(pm.faces))
                q = trimesh.Trimesh(v2, f2, process=True)
                if np.abs(q.bounds - pm.bounds).max() <= 0.002: break
                if small: q = pm.bounding_box_oriented.to_mesh(); break
                budget *= 2; q = pm
            out_parts.append(q)
        m = trimesh.util.concatenate(out_parts)
        bounds0 = np.array([np.min([pm.bounds[0] for pm in meshes], 0), np.max([pm.bounds[1] for pm in meshes], 0)])
        shrink = np.abs(m.bounds - bounds0).max()
        if shrink > 0.002:
            print('  note: %s outline moved by %.1f mm when simplified' % (g['name'], shrink * 1000))
        m.export(os.path.join(a.out, 'meshes', g['name'] + '.stl'))
        info['groups'][g['name']] = {'parts': len(keys), 'tris_cad': before, 'tris': len(m.faces), 'moving': bool(g.get('moving')),
                                     'bounds': m.bounds.round(4).tolist()}
        allV.append(m.vertices if not g.get('moving') else m.vertices @ rot_y(spec['hinge']['tilt_in_cad_deg']).T + hinge)
        print('%-14s %3d parts  %7d -> %5d triangles  %s' % (g['name'], len(keys), before, len(m.faces), 'tilts' if g.get('moving') else ''))

    if spec.get('rotors'):                                           # drawn rotors (no blades in the CAD)
        R = spec['rotors']; parts = []
        for cx, cz in R['centres_cad']:
            centre = to_ros([cx, R['height_cad'], cz])
            for k in range(R['blades']):
                ang = 2 * np.pi * k / R['blades']
                # a tapered blade: wide near the hub, narrow at the tip, slightly twisted (thin box per segment)
                for r0, r1, w in ((0.012, 0.05, R['chord']), (0.05, R['radius'], R['chord'] * 0.65)):
                    b = trimesh.creation.box([r1 - r0, w, 0.0025])
                    b.apply_translation([(r0 + r1) / 2, 0, 0])
                    b.apply_transform(trimesh.transformations.rotation_matrix(np.radians(8), [1, 0, 0]))
                    b.apply_transform(trimesh.transformations.rotation_matrix(ang, [0, 0, 1]))
                    b.apply_translation(centre); parts.append(b)
            hub = trimesh.creation.cylinder(radius=0.012, height=0.008, sections=16); hub.apply_translation(centre); parts.append(hub)
        m = trimesh.util.concatenate(parts)
        m.export(os.path.join(a.out, 'meshes', 'props.stl'))
        info['groups']['props'] = {'parts': len(R['centres_cad']), 'tris_cad': 0, 'tris': len(m.faces), 'moving': False,
                                   'bounds': m.bounds.round(4).tolist(), 'drawn': True}
        print('%-14s %3d rotors drawn, %d triangles (not in the CAD)' % ('props', len(R['centres_cad']), len(m.faces)))

    # labelled components for the drone selector: a point in base_link (or in the hinge frame, for tilting parts)
    comps = []
    for c in spec.get('components', []):
        if 'group' in c and c['group'] in info['groups']:
            gi = info['groups'][c['group']]; lo, hi = np.array(gi['bounds'])
            p = [(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, hi[2]]        # top centre of the group
            moving = gi['moving']
        else:
            p = to_ros(c['at_cad']); moving = False
        comps.append({k: v for k, v in c.items() if k not in ('at_cad', 'group')} | {'xyz': np.round(p, 4).tolist(), 'moving': bool(moving)})
    info['components'] = comps

    L = spec['lidar_link']
    base = to_ros(L['base_centre_cad'])
    lidar_level = (base - hinge) @ untilt.T + np.array([0, 0, L.get('origin_above_base_m', 0.0)])
    info['lidar_from_hinge'] = lidar_level.round(5).tolist()             # lidar_link in the hinge frame at zero tilt
    info['lidar_origin_confirmed'] = False if L.get('origin_above_base_m', 0.0) == 0.0 else True
    info['camera_xyz'] = to_ros(spec['camera_link']['centre_cad']).round(5).tolist()
    B = np.vstack(allV); info['bounds'] = [B.min(0).round(4).tolist(), B.max(0).round(4).tolist()]
    t = rot_y(spec['hinge']['tilt_in_cad_deg'])
    info['lidar_xyz_at_cad_tilt'] = (hinge + t @ lidar_level).round(5).tolist()
    json.dump(info, open(os.path.join(a.out, 'model.json'), 'w'), indent=1)
    print('hinge (base_link)', hinge.round(4), ' lidar_link from hinge at 0 deg', lidar_level.round(4),
          ' lidar_link at %.1f deg' % spec['hinge']['tilt_in_cad_deg'], info['lidar_xyz_at_cad_tilt'])
    print('model bounds', info['bounds'])


if __name__ == '__main__':
    main()
