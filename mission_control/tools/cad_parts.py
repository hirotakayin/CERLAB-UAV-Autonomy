#!/usr/bin/env python3
"""Read a folder of STEP parts (one file per part, as Onshape exports an assembly), mesh each one coarsely and save
them as one .npz (vertices and faces per part, in metres), with a table of every part: name, size, centre, triangles.
Identical parts (same size and triangle count) are reported once with their count, so the table stays readable.

  cad_parts.py <step folder> <out.npz> [--deflection 0.5]     (deflection in mm: bigger = coarser)

Run with the CAD environment: ~/drone_cad/.venv/bin/python (cadquery-ocp, trimesh).
"""
import argparse
import glob
import os
import re

import numpy as np
from OCP.BRep import BRep_Tool
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.IFSelect import IFSelect_RetDone
from OCP.STEPControl import STEPControl_Reader
from OCP.TopAbs import TopAbs_FACE
from OCP.TopExp import TopExp_Explorer
from OCP.TopLoc import TopLoc_Location
from OCP.TopoDS import TopoDS
from OCP.Interface import Interface_Static


def load_step(path, deflection_mm):
    Interface_Static.SetCVal_s('xstep.cascade.unit', 'MM')   # everything comes out in mm
    r = STEPControl_Reader()
    if r.ReadFile(path) != IFSelect_RetDone:
        raise RuntimeError('cannot read ' + path)
    r.TransferRoots()
    shape = r.OneShape()
    BRepMesh_IncrementalMesh(shape, deflection_mm, False, 0.5, True)
    V, F, n = [], [], 0
    ex = TopExp_Explorer(shape, TopAbs_FACE)
    while ex.More():
        face = TopoDS.Face_s(ex.Current())
        loc = TopLoc_Location()
        tri = BRep_Tool.Triangulation_s(face, loc)
        if tri is not None:
            T = loc.Transformation()
            pts = []
            for i in range(1, tri.NbNodes() + 1):
                p = tri.Node(i).Transformed(T)
                pts.append((p.X(), p.Y(), p.Z()))
            rev = face.Orientation() == 1                      # TopAbs_REVERSED
            for i in range(1, tri.NbTriangles() + 1):
                a, b, c = tri.Triangle(i).Get()
                F.append((a - 1 + n, c - 1 + n, b - 1 + n) if rev else (a - 1 + n, b - 1 + n, c - 1 + n))
            V += pts; n += len(pts)
        ex.Next()
    return np.array(V, float).reshape(-1, 3) / 1000.0, np.array(F, np.int32).reshape(-1, 3)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('folder'); ap.add_argument('out'); ap.add_argument('--deflection', type=float, default=0.5)
    a = ap.parse_args()
    parts = {}
    for path in sorted(glob.glob(os.path.join(a.folder, '*.step')) + sorted(glob.glob(os.path.join(a.folder, '*.STEP')))):
        name = re.sub(r'^Assembly \d+ - ', '', os.path.splitext(os.path.basename(path))[0])
        try:
            V, F = load_step(path, a.deflection)
        except Exception as e:
            print('skip', name, e); continue
        parts[name] = (V, F)
    np.savez_compressed(a.out, **{k + '|V': v[0] for k, v in parts.items()}, **{k + '|F': v[1] for k, v in parts.items()})
    allV = np.vstack([v[0] for v in parts.values()])
    print('%d parts, %d triangles; assembly box (m): min %s max %s' % (len(parts), sum(len(v[1]) for v in parts.values()),
          allV.min(0).round(3), allV.max(0).round(3)))
    groups = {}
    for k, (V, F) in parts.items():
        size = tuple(np.ptp(V, 0).round(3)) if len(V) else (0, 0, 0)
        groups.setdefault((size, len(F)), []).append((k, V.mean(0).round(3) if len(V) else None))
    print('%-34s %4s %-24s %-26s %7s' % ('part (first of identical)', 'n', 'size x y z (m)', 'centre (first)', 'tris'))
    for (size, nt), items in sorted(groups.items(), key=lambda g: -np.prod(np.maximum(g[0][0], 1e-3))):
        print('%-34s %4d %-24s %-26s %7d' % (items[0][0][:34], len(items), str(size), str(items[0][1]), nt))


if __name__ == '__main__':
    main()
