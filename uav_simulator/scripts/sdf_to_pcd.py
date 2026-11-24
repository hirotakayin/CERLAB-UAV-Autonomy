#!/usr/bin/env python3
"""Point cloud of a box-only world model, for EPIC's MARSIM simulator: every box face sampled on a grid.

  sdf_to_pcd.py models/excavation_slab/excavation_slab.sdf ~/EPIC/src/MARSIM/map_generator/resource/excavation_slab.pcd

Points inside another box are dropped. Boxes must be axis aligned, as in the excavation models.
"""
import re
import sys

import numpy as np

STEP = 0.05 # m, as in excavation1.pcd


def boxes(path):
    text = open(path).read()
    out = []
    for block in re.findall(r'<collision name=[^>]*>(.*?)</collision>', text, re.S):
        pose = re.search(r'<pose>(.*?)</pose>', block)
        size = re.search(r'<size>(.*?)</size>', block)
        if not pose or not size: continue
        p = [float(v) for v in pose.group(1).split()]
        if any(abs(a) > 1e-6 for a in p[3:6]): raise SystemExit('rotated box: not supported')
        c, s = np.array(p[:3]), np.array([float(v) for v in size.group(1).split()])
        out.append((c - s / 2, c + s / 2))
    return out


def face_points(lo, hi):
    pts = []
    for axis in range(3):
        u, v = [a for a in range(3) if a != axis]
        gu = np.arange(lo[u], hi[u] + STEP / 2, STEP)
        gv = np.arange(lo[v], hi[v] + STEP / 2, STEP)
        U, V = np.meshgrid(gu, gv, indexing='ij')
        for w in (lo[axis], hi[axis]):
            f = np.empty((U.size, 3))
            f[:, u], f[:, v], f[:, axis] = U.ravel(), V.ravel(), w
            pts.append(f)
    return np.vstack(pts)


def main(sdf, pcd):
    bs = boxes(sdf)
    clouds = []
    for i, (lo, hi) in enumerate(bs):
        pts = face_points(lo, hi)
        keep = np.ones(len(pts), bool)
        for j, (lo2, hi2) in enumerate(bs):
            if j != i: keep &= ~np.all((pts > lo2 + 1e-3) & (pts < hi2 - 1e-3), axis=1)
        clouds.append(pts[keep])
    cloud = np.unique(np.round(np.vstack(clouds), 3), axis=0).astype('<f4')
    with open(pcd, 'wb') as f:
        f.write(('# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
                 'WIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA binary\n' % (len(cloud), len(cloud))).encode())
        f.write(cloud.tobytes())
    print('%s: %d boxes, %d points, x %.1f..%.1f y %.1f..%.1f z %.1f..%.1f' % (pcd, len(bs), len(cloud), *[v for pair in zip(cloud.min(0), cloud.max(0)) for v in pair]))


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2])
