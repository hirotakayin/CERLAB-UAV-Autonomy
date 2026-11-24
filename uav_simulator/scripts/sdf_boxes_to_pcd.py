#!/usr/bin/env python3
"""Convert the <box> collision geometry of a Gazebo SDF model into a PCD map.

Samples the surface of every box collision on a regular grid and writes a
binary PCD (x y z float32) that MARSIM's map_generator can load.

Only axis-aligned boxes with a translation-only pose are supported (non-zero
roll/pitch/yaw is applied, but meshes, cylinders and spheres are skipped).

Usage:
  python3 sdf_boxes_to_pcd.py ../models/excavation1/env_one.sdf excavation1.pcd --res 0.05
"""
import argparse
import xml.etree.ElementTree as ET

import numpy as np


def rpy_to_matrix(roll, pitch, yaw):
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def parse_pose(elem):
    if elem is None or not elem.text:
        return np.zeros(6)
    return np.array([float(v) for v in elem.text.split()])


def sample_box_surface(size, res):
    """Return points on the six faces of a box centred at the origin."""
    half = np.asarray(size) / 2.0
    axes = [np.arange(-h, h + 1e-9, res) if h > 0 else np.array([0.0]) for h in half]
    # make sure the edges are included even when size is not a multiple of res
    axes = [np.unique(np.append(a, [-h, h])) for a, h in zip(axes, half)]
    faces = []
    for fixed in range(3):
        free = [i for i in range(3) if i != fixed]
        u, v = np.meshgrid(axes[free[0]], axes[free[1]], indexing="ij")
        for sign in (-1.0, 1.0):
            pts = np.empty((u.size, 3))
            pts[:, fixed] = sign * half[fixed]
            pts[:, free[0]] = u.ravel()
            pts[:, free[1]] = v.ravel()
            faces.append(pts)
    return np.vstack(faces)


def write_pcd(path, points):
    points = points.astype(np.float32)
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
        f"WIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {len(points)}\nDATA binary\n"
    )
    with open(path, "wb") as f:
        f.write(header.encode("ascii"))
        f.write(points.tobytes())


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("sdf", help="model SDF file")
    parser.add_argument("pcd", help="output PCD file")
    parser.add_argument("--res", type=float, default=0.05, help="surface sampling resolution in metres")
    parser.add_argument("--offset", type=float, nargs=3, default=[0.0, 0.0, 0.0],
                        help="extra x y z offset applied to every point (the model's pose in the world)")
    args = parser.parse_args()

    root = ET.parse(args.sdf).getroot()
    clouds = []
    for link in root.iter("link"):
        link_pose = parse_pose(link.find("pose"))
        R_link = rpy_to_matrix(*link_pose[3:])
        for col in link.findall("collision"):
            box = col.find("geometry/box/size")
            if box is None:
                print(f"skipping non-box collision: {col.get('name')}")
                continue
            size = [float(v) for v in box.text.split()]
            pose = parse_pose(col.find("pose"))
            R = R_link @ rpy_to_matrix(*pose[3:])
            t = R_link @ pose[:3] + link_pose[:3]
            pts = sample_box_surface(size, args.res) @ R.T + t
            clouds.append(pts)
            print(f"{col.get('name'):35s} size {size} -> {len(pts)} points")

    points = np.vstack(clouds) + np.asarray(args.offset)
    # drop duplicates from shared edges
    points = np.unique(np.round(points / (args.res / 2)).astype(np.int64), axis=0) * (args.res / 2)
    write_pcd(args.pcd, points)
    lo, hi = points.min(axis=0), points.max(axis=0)
    print(f"wrote {len(points)} points to {args.pcd}")
    print(f"bounds x [{lo[0]:.2f}, {hi[0]:.2f}]  y [{lo[1]:.2f}, {hi[1]:.2f}]  z [{lo[2]:.2f}, {hi[2]:.2f}]")


if __name__ == "__main__":
    main()
