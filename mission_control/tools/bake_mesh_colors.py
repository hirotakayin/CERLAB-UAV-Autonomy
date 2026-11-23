#!/usr/bin/env python3
"""A textured COLLADA mesh (one <triangles> block with UVs and one base-colour image, as Blender exports) as a colour
STL for the drone previews: each face takes the texture's colour at the centroid of its UVs, stored the Materialise way
(header "COLOR=", 15-bit RGB per face), which three.js's STLLoader reads as vertex colours. No textures to load in the
page, and the geometry stays the DAE's (Gazebo draws the same mesh).

  tools/bake_mesh_colors.py uav_simulator/urdf/quadcopter/meshes/CERLAB_quadcopter.dae web/meshes/CERLAB_quadcopter.stl

Needs numpy and Pillow (system python3).
"""
import os
import re
import struct
import sys
import urllib.parse

import numpy as np
from PIL import Image

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from assets import STL_RECORD  # noqa: E402


def floats(text, sid):
    m = re.search(r'<source id="%s".*?<float_array[^>]*>([^<]*)</float_array>' % re.escape(sid), text, re.S)
    return np.array(m.group(1).split(), np.float32)


def main(dae, out):
    text = open(dae, errors='replace').read()
    tri = re.search(r'<triangles[^>]*count="(\d+)"[^>]*>(.*?)</triangles>', text, re.S)
    n, body = int(tri.group(1)), tri.group(2)
    inputs = re.findall(r'<input semantic="(\w+)" source="#([^"]+)" offset="(\d+)"', body)
    stride = max(int(o) for _, _, o in inputs) + 1
    src = {sem: (s, int(o)) for sem, s, o in inputs}
    vert_src = re.search(r'<vertices id="%s">.*?<input semantic="POSITION" source="#([^"]+)"' % re.escape(src['VERTEX'][0]), text, re.S).group(1)
    P = floats(text, vert_src).reshape(-1, 3)
    UV = floats(text, src['TEXCOORD'][0]).reshape(-1, 2)
    idx = np.array(re.search(r'<p>([^<]*)</p>', body).group(1).split(), np.int64).reshape(n, 3, stride)
    V = P[idx[:, :, src['VERTEX'][1]]]                                  # (n, 3, 3)
    uv = UV[idx[:, :, src['TEXCOORD'][1]]].mean(1) % 1.0                # face centroid in texture space
    img_name = urllib.parse.unquote(re.findall(r'<init_from>([^<]*\.(?:png|jpg|jpeg))</init_from>', text, re.I)[0])
    tex = np.asarray(Image.open(os.path.join(os.path.dirname(dae), img_name)).convert('RGB'))
    h, w = tex.shape[:2]
    rgb = tex[np.clip(((1 - uv[:, 1]) * h).astype(int), 0, h - 1), np.clip((uv[:, 0] * w).astype(int), 0, w - 1)]
    c5 = (rgb.astype(np.uint16) * 31 + 127) // 255                       # 5 bits a channel; bit 15 clear = use this colour
    attr = c5[:, 0] | (c5[:, 1] << 5) | (c5[:, 2] << 10)
    nrm = np.cross(V[:, 1] - V[:, 0], V[:, 2] - V[:, 0]); nrm /= np.linalg.norm(nrm, axis=1, keepdims=True) + 1e-12
    rec = np.zeros(n, STL_RECORD)
    rec['n'], rec['v'], rec['a'] = nrm, V, attr
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    with open(out, 'wb') as f:
        f.write(b'COLOR=' + bytes([128, 128, 128, 255]) + b' baked from %s' % os.path.basename(dae).encode()[:60])
        f.seek(80); f.write(struct.pack('<I', n)); f.write(rec.tobytes())
    share = {k: round(float(np.mean(m)), 3) for k, m in (('dark', rgb.max(1) < 60), ('red', (rgb[:, 0] > 150) & (rgb[:, 1] < 90)),
                                                         ('light', rgb.min(1) > 180))}
    print('wrote %s: %d faces, bounds %s, colour shares %s' % (out, n, np.round([V.reshape(-1, 3).min(0), V.reshape(-1, 3).max(0)], 3).tolist(), share))


if __name__ == '__main__':
    main(*sys.argv[1:3])
