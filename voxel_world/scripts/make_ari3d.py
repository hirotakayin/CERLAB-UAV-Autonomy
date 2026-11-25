#!/usr/bin/env python3
"""3D training worlds from ARiADNE map PNGs (voxel_world.ari3d), one folder per world, with previews.

Needs scipy and scikit-image: run with ~/.venvs/ariadne/bin/python.

  make_ari3d.py --count 8 --seed 0 --out ~/mission_runs/ari3d/train_preview
  make_ari3d.py --pngs maps/img_1609.png maps/627.png --out /tmp/w --pixel 0.4      # double ARiADNE's scale

Each folder <png stem>_s<seed>/ holds world.npz, stats.json and preview.png; index.json in --out lists them.
--seed picks the PNGs (when --pngs is not given) and the heights: world k uses seed + k.
"""
import argparse
import glob
import json
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src'))
from voxel_world.ari3d import Ari3D, Ari3DParams  # noqa: E402

MAPS = os.path.expanduser('~/rl/large-scale-DRL-exploration/maps')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--out', required=True)
    ap.add_argument('--pngs', nargs='*', help='map PNGs (default: --count drawn from --maps)')
    ap.add_argument('--maps', default=MAPS)
    ap.add_argument('--count', type=int, default=4)
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--pixel', type=float, default=Ari3DParams.pixel, help="m per PNG pixel (0.2 = ARiADNE's own scale)")
    ap.add_argument('--flat', action='store_true', help='no height variation: the 2D map extruded (sanity check)')
    ap.add_argument('--no-preview', action='store_true')
    a = ap.parse_args()

    pngs = a.pngs or list(np.random.default_rng(a.seed).choice(sorted(glob.glob(os.path.join(a.maps, '*.png'))), a.count,
                                                              replace=False))
    params = Ari3DParams(pixel=a.pixel)
    if a.flat:
        params = Ari3DParams(pixel=a.pixel, level_change=0.0, clear=(8.0, 8.0), sunken=(0.0, 3.0, 6.0),
                             low_island=(0.0, 1.5), overhang=(0.0, 1.6, 4.0), max_gates=0, max_flat_reach=1.0,
                             max_staggers=0, min_staggers=0, max_bridges=0)
    out = os.path.expanduser(a.out)
    os.makedirs(out, exist_ok=True)
    index = []
    for k, png in enumerate(pngs):
        tic = time.time()
        seed = a.seed + k
        try:
            w = Ari3D(png, seed=seed, params=params)
        except (ValueError, RuntimeError) as err:  # e.g. the start is not in free configuration space
            print('%-24s skipped: %s' % ('%s_s%d' % (os.path.splitext(os.path.basename(png))[0], seed), err), flush=True)
            continue
        folder = os.path.join(out, '%s_s%d' % (os.path.splitext(os.path.basename(png))[0], seed))
        w.save(folder)
        if not a.no_preview:
            w.preview(os.path.join(folder, 'preview.png'))
        index.append(dict(folder=os.path.basename(folder), **w.info))
        i = w.info
        print('%-24s %4.1f s  %s m  %d regions, %d levels (span %.1f m), %d pits  %d islands (%d low, %d lips)  '
              '%d hurdles %d lintels  %d staggers  %d bridges  single-height reach %.0f%%  fixes %d' % (
                  os.path.basename(folder), time.time() - tic, 'x'.join('%.0f' % v for v in i['size_m']), i['regions'],
                  i['levels'], i['level_span_m'], i['sunken'], i['islands'], i['low_islands'], i['lips'], i['hurdles'],
                  i['lintels'], i['staggers'], i['bridges'], 100 * i['flat_reach'], i['flattened']), flush=True)
    # merge with an existing index, so several runs into one --out keep every world listed
    path = os.path.join(out, 'index.json')
    if os.path.exists(path):
        with open(path) as f:
            mine = {r['folder'] for r in index}
            index = [r for r in json.load(f) if r.get('folder') not in mine] + index
    index.sort(key=lambda r: r['folder'])
    with open(path, 'w') as f:
        json.dump(index, f, indent=1)


if __name__ == '__main__':
    main()
