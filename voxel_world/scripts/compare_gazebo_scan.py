#!/usr/bin/env python3
"""Compare predicted Mid-360 ranges with a synchronized Gazebo point cloud."""
import argparse
import csv
import json
from pathlib import Path
import sys

import numpy as np
import message_filters
import rospkg
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2

from voxel_world import VoxelWorld
from voxel_world.world import (lidar_pose_from_body, load_mid360_directions,
                               rotation_from_quaternion)


class Comparison:
    def __init__(self, args):
        rp = rospkg.RosPack()
        sim = Path(rp.get_path('uav_simulator'))
        model = args.model or sim / 'models/excavation_long/excavation_long.sdf'
        pattern = args.pattern or sim / 'src/livox_lidar/scan_mode/mid360-real-centr.csv'
        self.world = VoxelWorld.from_sdf(model, args.origin, args.size, args.resolution)
        self.pattern_directions = load_mid360_directions(pattern, args.downsample, samples=None)
        self.full_ray_count = int(np.ceil(args.samples / args.downsample))
        self.pattern_frame_count = max(1, len(self.pattern_directions) // self.full_ray_count)
        if args.rays > 0 and args.rays < self.full_ray_count:
            self.ray_ids = np.linspace(0, self.full_ray_count - 1, args.rays).astype(int)
        else:
            self.ray_ids = np.arange(self.full_ray_count)
        self.args = args
        self.rows = []
        self.scans = 0
        cloud = message_filters.Subscriber(args.cloud_topic, PointCloud2, queue_size=4)
        odom = message_filters.Subscriber(args.odom_topic, Odometry, queue_size=20)
        self.sync = message_filters.ApproximateTimeSynchronizer([cloud, odom], 20, args.sync_slop)
        self.sync.registerCallback(self.callback)
        rospy.loginfo('Voxel map shape [z,y,x]=%s; occupied voxels=%d',
                      self.world.occupied.shape, int(self.world.occupied.sum()))
        rospy.loginfo('Comparing %d of %d ordered Livox rays per scan across %d pattern frames, map resolution %.3f m',
                      len(self.ray_ids), self.full_ray_count,
                      self.pattern_frame_count, args.resolution)

    def callback(self, cloud, odom):
        points = np.asarray(list(point_cloud2.read_points(
            cloud, field_names=('x', 'y', 'z'), skip_nans=False)), dtype=np.float64)
        if points.ndim != 2 or points.shape[1] != 3:
            rospy.logwarn_throttle(5.0, 'Could not decode XYZ fields from point cloud')
            return
        if points.shape[0] != self.full_ray_count:
            rospy.logwarn_throttle(
                5.0, 'Livox point count %d differs from configured full scan ray count %d; skipping scan',
                points.shape[0], self.full_ray_count)
            return

        p = odom.pose.pose.position
        q = odom.pose.pose.orientation
        frame = (int(round(cloud.header.stamp.to_sec() * self.args.scan_rate)) +
                 self.args.scan_index_offset) % self.pattern_frame_count
        start = frame * self.full_ray_count
        direction_ids = (start + self.ray_ids) % len(self.pattern_directions)
        directions = self.pattern_directions[direction_ids]
        body_rotation = rotation_from_quaternion(q.x, q.y, q.z, q.w)
        sensor_origin, sensor_rotation = lidar_pose_from_body(
            [p.x, p.y, p.z], body_rotation, self.args.lidar_pitch_deg, self.args.sensor_z)
        world_directions = directions @ sensor_rotation.T

        expected = np.empty(len(world_directions), dtype=np.float64)
        for i, direction in enumerate(world_directions):
            hit_range = self.world.raycast(sensor_origin, direction, self.args.max_range)
            expected[i] = np.nan if hit_range is None else hit_range

        actual = np.linalg.norm(points[self.ray_ids], axis=1)
        actual[(~np.isfinite(actual)) | (actual < 0.1) | (actual >= self.args.max_range)] = np.nan
        expected[np.isfinite(expected) & (expected >= self.args.max_range)] = np.nan
        both = np.isfinite(expected) & np.isfinite(actual)
        expected_hit = np.isfinite(expected)
        actual_hit = np.isfinite(actual)
        union = np.count_nonzero(expected_hit | actual_hit)
        hit_iou = (np.count_nonzero(expected_hit & actual_hit) / union) if union else 1.0
        errors = np.abs(expected[both] - actual[both])
        row = {
            'scan': self.scans,
            'stamp_s': cloud.header.stamp.to_sec(),
            'pose_xyz': [p.x, p.y, p.z],
            'rays': int(len(expected)),
            'pattern_frame': int(frame),
            'expected_hits': int(expected_hit.sum()),
            'gazebo_hits': int(actual_hit.sum()),
            'hit_iou': float(hit_iou),
            'matched_hits': int(both.sum()),
            'range_mae_m': float(errors.mean()) if errors.size else None,
            'range_median_abs_error_m': float(np.median(errors)) if errors.size else None,
            'range_p95_abs_error_m': float(np.percentile(errors, 95)) if errors.size else None,
        }
        self.rows.append((row, expected, actual))
        rospy.loginfo('scan %d: hits expected/Gazebo %d/%d, hit IoU %.3f, range MAE %s m',
                      row['scan'], row['expected_hits'], row['gazebo_hits'], row['hit_iou'],
                      'n/a' if row['range_mae_m'] is None else '%.3f' % row['range_mae_m'])
        self.scans += 1
        if self.scans >= self.args.scans:
            self.finish()

    def finish(self):
        if self.args.output_csv:
            with open(self.args.output_csv, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['scan', 'ray_index', 'expected_range_m', 'gazebo_range_m'])
                for row, expected, actual in self.rows:
                    for ray_id, exp_range, act_range in zip(self.ray_ids, expected, actual):
                        writer.writerow([row['scan'], int(ray_id), exp_range, act_range])
        print(json.dumps([r[0] for r in self.rows], indent=2))
        rospy.signal_shutdown('requested scan count reached')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model')
    parser.add_argument('--pattern')
    parser.add_argument('--cloud-topic', default='/cloud_registered_body')
    parser.add_argument('--odom-topic', default='/CERLAB/quadcopter/odom')
    parser.add_argument('--origin', nargs=3, type=float, default=[-23.0, -45.0, -10.5])
    parser.add_argument('--size', nargs=3, type=float, default=[46.0, 90.0, 15.0])
    parser.add_argument('--resolution', type=float, default=0.2)
    parser.add_argument('--lidar-pitch-deg', type=float, default=23.5)
    parser.add_argument('--sensor-z', type=float, default=0.13)
    parser.add_argument('--max-range', type=float, default=20.0)
    parser.add_argument('--samples', type=int, default=20000)
    parser.add_argument('--downsample', type=int, default=2)
    parser.add_argument('--scan-rate', type=float, default=10.0,
                        help='Gazebo Livox update rate used to select the CSV pattern frame')
    parser.add_argument('--scan-index-offset', type=int, default=-1,
                        help='frame offset; plugin starts at row zero on its first scan')
    parser.add_argument('--rays', type=int, default=2048,
                        help='uniformly sample this many ordered rays; 0 checks every ray')
    parser.add_argument('--scans', type=int, default=1)
    parser.add_argument('--sync-slop', type=float, default=0.05)
    parser.add_argument('--output-csv')
    args, _ = parser.parse_known_args([a for a in sys.argv[1:] if ':=' not in a])
    rospy.init_node('voxel_gazebo_scan_compare', anonymous=True)
    Comparison(args)
    rospy.spin()


if __name__ == '__main__':
    main()
