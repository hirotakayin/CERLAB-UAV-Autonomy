#!/usr/bin/env python3
"""Write the scan pattern of the ideal 360 depth sensor (quadcopter_omni.urdf) for the Livox ray plugin:
src/livox_lidar/scan_mode/omni360-ideal.csv, in the Mid-360 file's format (Time/s, Azimuth/deg, Zenith/deg; zenith 90 is
horizontal).

Each scan is a Fibonacci sphere, turned by a different random rotation so that successive scans fill each other's gaps.
Rays inside the cone straight down that the drone's own body blocks (half-angle --blind) are left out, as the camera
cannot see through the drone either.

  make_omni_pattern.py [--per-scan 20000] [--scans 10] [--blind 35]
"""
import argparse
import os

import numpy as np

ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument('--per-scan', type=int, default=20000, help='rays per scan, the plugin <samples>')
ap.add_argument('--scans', type=int, default=10, help='differently rotated spheres before the pattern repeats')
ap.add_argument('--blind', type=float, default=35.0, help='deg, half-angle of the cone below the drone left out')
a = ap.parse_args()
rng = np.random.default_rng(0)
golden = np.pi * (3 - np.sqrt(5))
rows = []
for s in range(a.scans):
    n = a.per_scan
    i = np.arange(n) + 0.5
    z = 1 - 2 * i / n
    r = np.sqrt(1 - z * z)
    p = np.column_stack([r * np.cos(golden * i), r * np.sin(golden * i), z])
    q, _ = np.linalg.qr(rng.normal(size=(3, 3)))  # random rotation
    p = p @ q.T
    keep = p[:, 2] > -np.cos(np.radians(a.blind))
    p = p[keep]
    az = np.degrees(np.arctan2(p[:, 1], p[:, 0]))
    zen = np.degrees(np.arccos(np.clip(p[:, 2], -1, 1)))
    rows.append(np.column_stack([az, zen]))
rows = np.vstack(rows)
out = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src', 'livox_lidar', 'scan_mode', 'omni360-ideal.csv')
with open(out, 'w') as f:
    f.write('Time/s,Azimuth/deg,Zenith/deg\n')
    for k, (az, zen) in enumerate(rows):
        f.write('%d,%.5f,%.5f\n' % (k + 1, az, zen))
print('Wrote %d rays to %s' % (len(rows), os.path.normpath(out)))
