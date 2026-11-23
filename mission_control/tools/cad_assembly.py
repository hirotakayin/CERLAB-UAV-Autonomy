#!/usr/bin/env python3
"""Read one assembly STEP (parts placed, with names), mesh every part instance coarsely and save them to an .npz in
metres, plus a table of the instances: name, path in the assembly tree, size and centre. The next step
(cad_to_model.py) groups and simplifies them.

  cad_assembly.py "Assembly 2.step" parts.npz [--deflection 0.5]     (deflection in mm: bigger = coarser)

Run with the CAD environment: ~/drone_cad/.venv/bin/python (cadquery-ocp).
"""
import argparse
import json

import numpy as np
from OCP.BRep import BRep_Tool
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.IFSelect import IFSelect_RetDone
from OCP.Interface import Interface_Static
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDataStd import TDataStd_Name
from OCP.TDF import TDF_Label, TDF_LabelSequence
from OCP.TDocStd import TDocStd_Document
from OCP.TopAbs import TopAbs_FACE, TopAbs_REVERSED
from OCP.TopExp import TopExp_Explorer
from OCP.TopLoc import TopLoc_Location
from OCP.TopoDS import TopoDS
from OCP.XCAFDoc import XCAFDoc_DocumentTool


def name_of(label):
    attr = TDataStd_Name()
    if label.FindAttribute(TDataStd_Name.GetID_s(), attr):
        return attr.Get().ToExtString()
    return '?'


def mesh(shape, deflection_mm):
    BRepMesh_IncrementalMesh(shape, deflection_mm, False, 0.5, True)
    V, F, n = [], [], 0
    ex = TopExp_Explorer(shape, TopAbs_FACE)
    while ex.More():
        face = TopoDS.Face_s(ex.Current())
        loc = TopLoc_Location()
        tri = BRep_Tool.Triangulation_s(face, loc)
        if tri is not None:
            T = loc.Transformation()
            for i in range(1, tri.NbNodes() + 1):
                p = tri.Node(i).Transformed(T); V.append((p.X(), p.Y(), p.Z()))
            rev = face.Orientation() == TopAbs_REVERSED
            for i in range(1, tri.NbTriangles() + 1):
                a, b, c = tri.Triangle(i).Get()
                F.append((a - 1 + n, c - 1 + n, b - 1 + n) if rev else (a - 1 + n, b - 1 + n, c - 1 + n))
            n = len(V)
        ex.Next()
    return np.array(V, float).reshape(-1, 3) / 1000.0, np.array(F, np.int32).reshape(-1, 3)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('step'); ap.add_argument('out'); ap.add_argument('--deflection', type=float, default=0.5)
    a = ap.parse_args()
    Interface_Static.SetCVal_s('xstep.cascade.unit', 'MM')
    doc = TDocStd_Document(TCollection_ExtendedString('doc'))
    r = STEPCAFControl_Reader(); r.SetNameMode(True)
    if r.ReadFile(a.step) != IFSelect_RetDone:
        raise SystemExit('cannot read ' + a.step)
    r.Transfer(doc)
    st = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())

    out, table = {}, []

    def walk(label, loc, path):
        shape_label = label
        if st.IsReference_s(label):
            ref = TDF_Label(); st.GetReferredShape_s(label, ref)
            loc = loc.Multiplied(st.GetLocation_s(label)); shape_label = ref
        nm = name_of(label) if st.IsReference_s(label) else name_of(shape_label)
        here = path + [nm]
        if st.IsAssembly_s(shape_label):
            kids = TDF_LabelSequence(); st.GetComponents_s(shape_label, kids)
            for i in range(1, kids.Length() + 1):
                walk(kids.Value(i), loc, here)
            return
        shape = st.GetShape_s(shape_label).Moved(loc)
        V, F = mesh(shape, a.deflection)
        k = '%03d %s' % (len(table), name_of(shape_label))
        out[k + '|V'], out[k + '|F'] = V.astype(np.float32), F
        table.append({'key': k, 'name': name_of(shape_label), 'path': '/'.join(here[1:-1]), 'tris': len(F),
                      'min': V.min(0).round(4).tolist() if len(V) else None, 'max': V.max(0).round(4).tolist() if len(V) else None})

    roots = TDF_LabelSequence(); st.GetFreeShapes(roots)
    for i in range(1, roots.Length() + 1):
        walk(roots.Value(i), TopLoc_Location(), [])
    out['table'] = np.frombuffer(json.dumps(table).encode(), np.uint8)
    np.savez_compressed(a.out, **out)
    V = np.vstack([out[t['key'] + '|V'] for t in table if t['tris']])
    print('%d instances, %d triangles; box (m) min %s max %s' % (len(table), sum(t['tris'] for t in table), V.min(0).round(3), V.max(0).round(3)))
    for t in table:
        if t['tris']:
            c = ((np.array(t['min']) + np.array(t['max'])) / 2).round(3); s = (np.array(t['max']) - np.array(t['min'])).round(3)
            print('%-28s %-26s size %-22s centre %s' % (t['name'][:28], t['path'][:26], s.tolist(), c.tolist()))


if __name__ == '__main__':
    main()
