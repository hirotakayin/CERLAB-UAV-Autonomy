#!/usr/bin/env python3
"""Drives the excavation site's machines (make_excavation_site.py): poses the excavator's house and arm and the crane's
load every step on sim time, and publishes their true shapes for scoring.

  out  /gazebo/set_model_state   excavator_house, excavator_arm, crane_load (kinematic models)
       /site/hazards             JSON at 10 Hz: {t, capsules: [[name, a xyz, b xyz, radius], ...] (rough, for drawing and
                                 phantom matching), boxes: [[name, centre, R 3x3, half extents], ...] and segments (the
                                 cable) (exact, for scoring clearance)}
Params (~): rate (50 Hz), t0 (sim s the machines start moving; before that they hold their first pose).
"""
import json
import os
import sys

import numpy as np
import rospy
from gazebo_msgs.msg import ModelState
from std_msgs.msg import String

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from make_excavation_site import SITE, crane_pose, excavator_pose, hazard_boxes, hazard_capsules  # noqa: E402


def quat(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = np.cos(roll / 2), np.sin(roll / 2), np.cos(pitch / 2), np.sin(pitch / 2), np.cos(yaw / 2), np.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


def state(name, xyz, rpy):
    m = ModelState(model_name=name, reference_frame='world')
    m.pose.position.x, m.pose.position.y, m.pose.position.z = xyz
    m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = quat(*rpy)
    return m


class SiteMotion:
    def __init__(self):
        self.t0 = rospy.get_param('~t0', 0.0)
        self.pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)
        self.hz_pub = rospy.Publisher('/site/hazards', String, queue_size=1)
        self.last_hz = -1.0
        rospy.Timer(rospy.Duration(1.0 / rospy.get_param('~rate', 50.0)), self.tick)

    def tick(self, _):
        now = rospy.Time.now().to_sec()
        t = max(now - self.t0, 0.0)
        e, c = SITE['excavator'], SITE['crane']
        yaw, el = excavator_pose(t)
        ex, ey = e['pos']
        self.pub.publish(state('excavator_house', (ex, ey, 0.0), (0, 0, yaw)))
        foot = (ex + e['foot'][0] * np.cos(yaw), ey + e['foot'][0] * np.sin(yaw), e['foot'][1])
        self.pub.publish(state('excavator_arm', foot, (0, -el, yaw)))  # Gazebo pitch: positive tips +x down
        sl, h = crane_pose(t)
        cx, cy = c['pos']
        self.pub.publish(state('crane_load', (cx + c['radius'] * np.cos(sl), cy + c['radius'] * np.sin(sl), h), (0, 0, sl)))
        if now - self.last_hz >= 0.1:
            self.last_hz = now
            caps = [[n, [round(float(v), 3) for v in a], [round(float(v), 3) for v in b], r] for n, a, b, r in hazard_capsules(t)]
            bx, segs = hazard_boxes(t)
            boxes = [[n, [round(float(v), 3) for v in c], np.round(R, 4).tolist(), [float(v) for v in h]] for n, c, R, h in bx]
            segs = [[n, [round(float(v), 3) for v in a], [round(float(v), 3) for v in b], r] for n, a, b, r in segs]
            self.hz_pub.publish(String(json.dumps(dict(t=now, capsules=caps, boxes=boxes, segments=segs))))


if __name__ == '__main__':
    rospy.init_node('site_motion')
    SiteMotion()
    rospy.spin()
