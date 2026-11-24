#!/usr/bin/env python3
"""URDF for a drone built from CAD (mission_control/tools/cad_to_model.py wrote urdf/<model>/meshes and model.json):
the Gazebo plugins and sensors of a template URDF, with
  - base_link drawn from the CAD meshes (frame, props, camera, mount), a box for collision;
  - lidar_hinge -> lidar_link: the tilting LiDAR and its bracket on a joint at the CAD hinge, angle = xacro arg
    lidar_pitch_deg, so every tilt config is the same file;
  - the Gazebo LiDAR placed where that joint puts lidar_link (sensor pose = hinge + R_y(tilt) * offset);
  - the depth camera at the CAD RealSense (camera_link, optical axes);
  - templates with the 360 camera (quadcopter_omni_lidar_cam.urdf): the Insta360 X4 drawn from its STEP
    (meshes/insta360_x4.stl, origin at the lens centre) and its two fisheyes at omni_link, placed by xacro args
    insta360_x / insta360_z (lens centre in base_link; the X4 is not in the drone CAD, so the defaults are --insta360)
    and insta360_sideways (0: lenses front and back, upright; 1: lenses left and right, the front lens facing +y, the
    lens centre on the drone's vertical axis (insta360_x ignored) and the X4 leaned forward insta360_side_tilt_deg,
    a fixed property, about +y at its lens centre). omni_link stays level at the lens centre (the panorama and ideal
    depth frame); insta360_front / insta360_back carry the lens rotations.
robot_state_publisher publishes base_link -> lidar_hinge -> lidar_link, camera_link and omni_link -> insta360_front|back
from it (start_world.launch does this for *_cad.urdf drones, instead of setupTF.launch's fixed offsets).

  make_cad_urdf.py cerlab_lidar_drone quadcopter_lidar.urdf quadcopter_cerlab_lidar_cad.urdf
  make_cad_urdf.py cerlab_lidar_drone quadcopter_omni_lidar_cam.urdf quadcopter_cerlab_lidar_cam_cad.urdf
"""
import json
import argparse
import os
import re

URDF = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'urdf')
# STL meshes carry no colour, and Gazebo 11 drops a URDF <material><color> (sdformat 9), so the CAD parts rendered
# white. Each group also gets an OBJ + MTL copy (meshes/gazebo/<group>.obj) with its colour, the way Mission Control
# draws it: carbon frame, grey guards, red printed mounts, white Mid-360, black cameras. The STLs stay the source (the
# GUI draws them); the URDF colour is kept for RViz.
COLOURS = {'frame': (0.23, 0.25, 0.27), 'guards': (0.33, 0.35, 0.37), 'props': (0.35, 0.38, 0.43), 'camera': (0.08, 0.09, 0.1),
           'mount': (0.61, 0.13, 0.15), 'lidar_bracket': (0.61, 0.13, 0.15), 'lidar_links': (0.61, 0.13, 0.15),
           'lidar_adapter': (0.17, 0.19, 0.21), 'lidar': (0.9, 0.91, 0.93), 'insta360_x4': (0.07, 0.07, 0.08)}


def colour_of(group):
    return COLOURS.get('props' if group.startswith('props') else group, COLOURS['frame'])


def colour(group):
    return '<material name="cad_%s"><color rgba="%.2f %.2f %.2f 1"/></material>' % ((group,) + colour_of(group))


def read_stl(path):
    """Triangles (n, 3, 3) of a binary STL."""
    import numpy as np
    raw = open(path, 'rb').read()
    n = int.from_bytes(raw[80:84], 'little')
    return np.frombuffer(raw[84:84 + 50 * n], np.dtype([('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')]))['v']


def write_gazebo_mesh(model, group):
    """meshes/gazebo/<group>.obj + .mtl: the STL with its colour (shared vertices, so the faces shade smoothly per facet)."""
    import numpy as np
    d = os.path.join(URDF, model, 'meshes', 'gazebo'); os.makedirs(d, exist_ok=True)
    T = read_stl(os.path.join(URDF, model, 'meshes', group + '.stl'))
    V, idx = np.unique(T.reshape(-1, 3).round(6), axis=0, return_inverse=True)
    F = idx.reshape(-1, 3) + 1
    r, g, b = colour_of(group)
    open(os.path.join(d, group + '.mtl'), 'w').write(
        'newmtl %s\nKa %.3f %.3f %.3f\nKd %.3f %.3f %.3f\nKs 0.15 0.15 0.15\nNs 20\nd 1\n' % (group, r, g, b, r, g, b))
    with open(os.path.join(d, group + '.obj'), 'w') as f:
        f.write('mtllib %s.mtl\nusemtl %s\n' % (group, group))
        f.write(''.join('v %.6f %.6f %.6f\n' % tuple(v) for v in V))
        f.write(''.join('f %d %d %d\n' % tuple(t) for t in F))


# Intel RealSense D435 (90 x 25 x 25 mm): depth origin 4.2 mm behind the front glass (datasheet). Assumed on the
# body's centreline; the real origin is the left imager, off the centreline in y, not identified in the CAD.
D435_ORIGIN_BEHIND_GLASS = 0.0042
# the Insta360 X4 in the left-right layout (insta360_sideways): leaned forward this much, top forward
SIDE_TILT_DEG = 55.0


def main(model, template, out, insta360):
    info = json.load(open(os.path.join(URDF, model, 'model.json')))
    t = open(os.path.join(URDF, template)).read()
    hx, hy, hz = info['hinge_xyz']
    lx, ly, lz = info['lidar_from_hinge']
    (x0, y0, z0), (x1, y1, z1) = info['bounds']
    for g in list(info['groups']) + (['insta360_x4'] if 'insta360_front' in t else []):
        write_gazebo_mesh(model, g)
    mesh = lambda g: '<mesh filename="package://uav_simulator/urdf/%s/meshes/gazebo/%s.obj"/>' % (model, g)
    fixed = [g for g, gi in info['groups'].items() if not gi['moving']]
    moving = [g for g, gi in info['groups'].items() if gi['moving']]
    org = lambda g: '%.4f %.4f %.4f' % tuple(info['groups'][g].get('origin', [0, 0, 0]))   # propellers sit on their hubs
    vis = lambda g: ('    <visual name="%s"><origin xyz="%s" rpy="0 0 0"/><geometry>%s</geometry>%s</visual>\n'
                     % (g, org(g), mesh(g), colour('props' if g.startswith('props') else g)))
    body = (
        '  <link name="base_link">\n'
        '    <inertial><mass value="1.54"/><origin rpy="0 0 0" xyz="0 0 0"/>'
        '<inertia ixx="0.1152" ixy="0" ixz="0" iyy="0.1152" iyz="0" izz="0.218"/></inertial>\n'
        '    <collision name="body"><origin xyz="%.4f %.4f %.4f" rpy="0 0 0"/><geometry><box size="%.4f %.4f %.4f"/></geometry></collision>\n'
        % ((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2, x1 - x0, y1 - y0, z1 - z0)
        + ''.join(vis(g) for g in fixed) +
        '  </link>\n'
        '  <!-- the LiDAR tilts about the front hinge of its mount; lidar_pitch_deg > 0 tips its top forward -->\n'
        '  <link name="lidar_hinge">\n' + ''.join(vis(g) for g in moving) +
        '    <inertial><mass value="0.001"/><origin xyz="0 0 0"/><inertia ixx="1e-6" ixy="0" ixz="0" iyy="1e-6" iyz="0" izz="1e-6"/></inertial>\n'
        '  </link>\n'
        '  <joint name="lidar_tilt" type="fixed"><parent link="base_link"/><child link="lidar_hinge"/>'
        '<origin xyz="%.5f %.5f %.5f" rpy="0 ${radians($(arg lidar_pitch_deg))} 0"/></joint>\n'
        '  <link name="lidar_link"/>\n'
        '  <joint name="lidar_mount" type="fixed"><parent link="lidar_hinge"/><child link="lidar_link"/>'
        '<origin xyz="%.5f %.5f %.5f" rpy="0 0 0"/></joint>\n' % (hx, hy, hz, lx, ly, lz))
    if not info.get('lidar_origin_confirmed'):
        body += '  <!-- lidar_link is on the MID-360 mounting face: Livox\'s point-cloud origin above it is not applied yet (model_spec.json) -->\n'
    # replace the template's base_link and point its LiDAR sensor at the hinge pose
    t = re.sub(r'  <link name="base_link">.*?</link>\n', lambda m: body, t, count=1, flags=re.S)
    pose = ('<pose>${%.5f + cos(radians($(arg lidar_pitch_deg))) * %.5f + sin(radians($(arg lidar_pitch_deg))) * %.5f} %.5f '
            '${%.5f - sin(radians($(arg lidar_pitch_deg))) * %.5f + cos(radians($(arg lidar_pitch_deg))) * %.5f} 0 ${radians($(arg lidar_pitch_deg))} 0</pose>'
            % (hx, lx, lz, hy + ly, hz, lx, lz))
    t, n = re.subn(r'<pose>0 0 0\.13 0 \$\{radians\(\$\(arg lidar_pitch_deg\)\)\} 0</pose>', lambda m: pose, t)
    if n != 1:
        raise SystemExit('template LiDAR pose not found (expected the 0.13 m tilted Mid-360 pose)')
    # depth camera: the CAD RealSense, facing +x; camera_link in optical axes (z forward, x right, y down)
    cx, cy, cz = info['groups']['camera']['bounds'][1][0] - D435_ORIGIN_BEHIND_GLASS, info['camera_xyz'][1], info['camera_xyz'][2]
    t, n = re.subn(r'<pose> 0\.09 0\.0 0\.095 0 0 0 </pose>', '<pose>%.4f %.4f %.4f 0 0 0</pose> <!-- CAD RealSense depth origin -->' % (cx, cy, cz), t)
    if n != 1:
        raise SystemExit('template depth camera pose not found (expected 0.09 0.0 0.095)')
    links = ('  <!-- RealSense depth origin (assumed on its centreline, %.1f mm behind the front glass), optical axes -->\n'
             '  <link name="camera_link"/>\n'
             '  <joint name="camera_mount" type="fixed"><parent link="base_link"/><child link="camera_link"/>'
             '<origin xyz="%.4f %.4f %.4f" rpy="-1.5707963 0 -1.5707963"/></joint>\n' % (1000 * D435_ORIGIN_BEHIND_GLASS, cx, cy, cz))
    # the 360 camera: xacro args so a test can move it without regenerating
    if 'insta360_front' in t:
        ix, iz = insta360
        t = t.replace('  <xacro:arg name="lidar_pitch_deg"',
                      '  <xacro:arg name="insta360_x" default="%.4f"/> <!-- Insta360 X4 lens centre in base_link (m) -->\n'
                      '  <xacro:arg name="insta360_z" default="%.4f"/>\n'
                      '  <xacro:arg name="insta360_sideways" default="0"/> <!-- 1: lenses left (front lens, +y) and right, on the axis, leaned forward -->\n'
                      '  <xacro:property name="insta360_side_tilt_deg" value="%g"/> <!-- sideways: turned about +y at the lens centre, top forward -->\n'
                      '  <xacro:arg name="lidar_pitch_deg"' % (ix, iz, SIDE_TILT_DEG), 1)
        sw = 'float($(arg insta360_sideways))'
        at = '${(1 - %s) * float($(arg insta360_x))} 0 $(arg insta360_z)' % sw          # sideways: on the drone's axis
        # lens -> base_link: R_y(tilt) R_z(yaw) as SDF/URDF rpy. Upright: front (0, 0, 0), back (0, 0, pi); sideways,
        # tilted t: front (t, 0, pi/2), back (-t, 0, -pi/2)
        tl = '%s * radians(insta360_side_tilt_deg)' % sw
        front = '${%s} 0 ${%s * pi / 2}' % (tl, sw)
        back = '${-%s} 0 ${pi - %s * 3 * pi / 2}' % (tl, sw)
        t, n0 = re.subn(r'(<sensor type="ray" name="omni360_ideal">\s*)<pose>0 0 0\.24 0 0 0</pose>', lambda m: m.group(1) + '<pose>%s 0 0 0</pose>' % at, t)
        t, n1 = re.subn(r'(<sensor name="insta360_front" type="wideanglecamera">\s*)<pose>0 0 0\.24 0 0 0</pose>', lambda m: m.group(1) + '<pose>%s %s</pose>' % (at, front), t)
        t, n2 = re.subn(r'(<sensor name="insta360_back" type="wideanglecamera">\s*)<pose>0 0 0\.24 0 0 3\.14159265</pose>', lambda m: m.group(1) + '<pose>%s %s</pose>' % (at, back), t)
        if (n0, n1, n2) != (1, 1, 1):
            raise SystemExit('template 360 camera poses not found (expected the ideal sensor and two fisheyes at 0 0 0.24)')
        links += ('  <!-- Insta360 X4: omni_link at the lens centre, level; the X4 and its lens frames turned by insta360_sideways -->\n'
                  '  <link name="omni_link">\n'
                  '    <visual name="insta360_x4"><origin xyz="0 0 0" rpy="%s"/><geometry>%s</geometry>%s</visual>\n'
                  '    <inertial><mass value="0.001"/><origin xyz="0 0 0"/><inertia ixx="1e-6" ixy="0" ixz="0" iyy="1e-6" iyz="0" izz="1e-6"/></inertial>\n'
                  '  </link>\n'
                  '  <joint name="insta360_mount" type="fixed"><parent link="base_link"/><child link="omni_link"/>'
                  '<origin xyz="%s" rpy="0 0 0"/></joint>\n'
                  '  <link name="insta360_front"/>\n'
                  '  <joint name="insta360_front_lens" type="fixed"><parent link="omni_link"/><child link="insta360_front"/>'
                  '<origin xyz="0 0 0" rpy="%s"/></joint>\n'
                  '  <link name="insta360_back"/>\n'
                  '  <joint name="insta360_back_lens" type="fixed"><parent link="omni_link"/><child link="insta360_back"/>'
                  '<origin xyz="0 0 0" rpy="%s"/></joint>\n' % (front, mesh('insta360_x4'), colour('insta360_x4'), at, front, back))
    t = re.sub(r'(  <joint name="lidar_mount".*?</joint>\n)', lambda m: m.group(1) + links, t, count=1, flags=re.S)
    t = t.replace('<robot name="quadcopter"', '<!-- generated by scripts/make_cad_urdf.py from %s and urdf/%s (CAD) -->\n<robot name="quadcopter"' % (template, model), 1)
    t = t.replace('base_link -> lidar_link is published in setupTF.launch', 'base_link -> lidar_link comes from this URDF (robot_state_publisher)')
    open(os.path.join(URDF, out), 'w').write(t)
    print('wrote', out)


if __name__ == '__main__':
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('model'); ap.add_argument('template'); ap.add_argument('out')
    ap.add_argument('--insta360', nargs=2, type=float, default=[-0.04, 0.344], metavar=('X', 'Z'),
                    help='default lens centre in base_link: 4 cm behind the centre, the X4\'s base 0.24 m above base_link '
                         '(lens 104.3 mm above its base); to be confirmed against the real mount')
    a = ap.parse_args()
    main(a.model, a.template, a.out, a.insta360)
