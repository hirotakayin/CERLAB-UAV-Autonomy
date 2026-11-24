#!/usr/bin/env python3
"""Ideal 360 optic-flow depth: keep only the points of the full-sphere sensor (/omni/cloud_raw) that a 360 camera could
range from optic flow at the drone's current velocity, and publish them for the map (/omni/cloud_flow, same frame).

Flow from translation is w = v sin(phi) / D (v speed, phi angle between the ray and the direction of travel, D distance).
With flow measured to sigma_w, the distance comes out to sigma_D / D = D sigma_w / (v sin(phi)). Keeping points whose
relative error is at most eps gives

    D <= (eps / sigma_w) * v * sin(phi) = flow_range_s * v * sin(phi)

so a hovering drone ranges nothing, and nothing straight ahead or behind (phi near 0 or 180 deg, the focus of expansion
and contraction). Rotation is assumed removed with the gyro, as a real pipeline would. The same flow_range_s is the
planner's omni_flow_range_s, so information gain predicts what this sensor will actually see.

Params (~): flow_range_s (s, default 10: eps 0.2 / sigma_w 0.02 rad/s), min_angle_deg (5), max_range (m, 20),
min_speed (m/s, 0.05: below this nothing is ranged), min_range (m, 0.6: the drone's own body), odom_topic, in_topic, out_topic.
"""
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField


class FlowLimitedCloud:
    def __init__(self):
        self.k = rospy.get_param('~flow_range_s', 10.0)
        self.min_sin = np.sin(np.radians(rospy.get_param('~min_angle_deg', 5.0)))
        self.max_range = rospy.get_param('~max_range', 20.0)
        self.min_speed = rospy.get_param('~min_speed', 0.05)
        self.min_range = rospy.get_param('~min_range', 0.6)  # the drone itself, masked out of the flow
        self.vel = np.zeros(3)  # body frame; omni_link is base_link raised 0.24 m, same axes
        self.pub = rospy.Publisher(rospy.get_param('~out_topic', '/omni/cloud_flow'), PointCloud2, queue_size=2)
        rospy.Subscriber(rospy.get_param('~odom_topic', '/CERLAB/quadcopter/odom_raw'), Odometry, self.on_odom, queue_size=5)
        rospy.Subscriber(rospy.get_param('~in_topic', '/omni/cloud_raw'), PointCloud2, self.on_cloud, queue_size=2,
                         buff_size=2 ** 24)
        rospy.loginfo('[flow_limited_cloud] D <= %.1f s * v * sin(phi), phi >= %.1f deg, D <= %.1f m',
                      self.k, np.degrees(np.arcsin(self.min_sin)), self.max_range)

    def on_odom(self, msg):
        t = msg.twist.twist.linear
        self.vel = np.array([t.x, t.y, t.z])

    def on_cloud(self, msg):
        offs = {f.name: f.offset for f in msg.fields}
        n = msg.width * msg.height
        raw = np.frombuffer(msg.data, np.uint8).reshape(n, msg.point_step)
        xyz = np.stack([raw[:, offs[c]:offs[c] + 4].copy().view(np.float32)[:, 0] for c in 'xyz'], 1)
        xyz = xyz[np.isfinite(xyz).all(1)]
        d = np.linalg.norm(xyz, axis=1)
        speed = np.linalg.norm(self.vel)
        if speed < self.min_speed or len(xyz) == 0:
            keep = np.zeros(len(xyz), bool)
        else:
            u = self.vel / speed
            cos = (xyz @ u) / np.maximum(d, 1e-6)
            sin = np.sqrt(np.clip(1 - cos * cos, 0, 1))
            keep = (sin >= self.min_sin) & (d <= self.k * speed * sin) & (d <= self.max_range) & (d > self.min_range)
        pts = xyz[keep].astype(np.float32)
        out = PointCloud2()
        out.header = msg.header
        out.height, out.width = 1, len(pts)
        out.fields = [PointField(c, 4 * i, PointField.FLOAT32, 1) for i, c in enumerate('xyz')]
        out.is_bigendian, out.point_step, out.row_step, out.is_dense = False, 12, 12 * len(pts), True
        out.data = pts.tobytes()
        self.pub.publish(out)


if __name__ == '__main__':
    rospy.init_node('flow_limited_cloud')
    FlowLimitedCloud()
    rospy.spin()
