#!/usr/bin/env python3
"""Voxelize a static SDF model. Defaults match the excavation_long scorer map."""
import argparse
from pathlib import Path
import sys

import rospkg

from voxel_world import VoxelWorld


def main():
    rp = rospkg.RosPack()
    sim = Path(rp.get_path('uav_simulator'))
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', default=str(sim / 'models/excavation_long/excavation_long.sdf'))
    parser.add_argument('--output', default='/tmp/excavation_long_voxels.npz')
    parser.add_argument('--origin', nargs=3, type=float, default=[-23.0, -45.0, -10.5], metavar=('X', 'Y', 'Z'))
    parser.add_argument('--size', nargs=3, type=float, default=[46.0, 90.0, 15.0], metavar=('X', 'Y', 'Z'))
    parser.add_argument('--resolution', type=float, default=0.2)
    args = parser.parse_args(rospy_free_argv(sys.argv[1:]))

    world = VoxelWorld.from_sdf(args.model, args.origin, args.size, args.resolution)
    world.save(args.output)
    print('model:', args.model)
    print('shape [z,y,x]:', tuple(int(v) for v in world.occupied.shape))
    print('resolution_m:', world.resolution)
    print('occupied_voxels:', int(world.occupied.sum()))
    print('dense_bytes:', world.occupied.nbytes)
    print('saved:', args.output)


def rospy_free_argv(argv):
    # Strip ROS remapping arguments if invoked with rosrun.
    return [arg for arg in argv if ':=' not in arg]


if __name__ == '__main__':
    main()
