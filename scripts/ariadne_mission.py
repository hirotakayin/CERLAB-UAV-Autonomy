#!/usr/bin/env python3
"""Flies the ARiADNE baseline in the Gazebo pits: takes off from the pit floor, follows ARiADNE's 2D waypoints at one
fixed height, then retraces its path home and lands where it started.

ARiADNE (marmotlab/ARiADNE-ROS-Planner, rl_planner) plans on a 2D slice of the map and only publishes (x, y) waypoints.
This node owns the tracking controller's target. In the authors' setup a local planner steers the robot to each
waypoint; here that role is a path on ARiADNE's own 2D map (/projected_map) with obstacles inflated by INFLATE, shortened
wherever a straight line is clear. The setpoint follows it at the mission speed and ~fly_z, nose into the direction of
travel so the tilted Mid-360 looks ahead. It speaks the same
state / event / plan topics as epic_mission.py, so epic_monitor_node scores it like EPIC and the CERLAB presets.
"""
import heapq
import json
import math

import numpy as np
import rospy
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from std_msgs.msg import Empty, Float32, String
from std_srvs.srv import Trigger, TriggerResponse
from tracking_controller.msg import Target

RATE = 50.0
CRUMB_SPACING = 0.25 # m between remembered positions on the flown path
CRUMB_LINK = 0.8 # m, remembered positions this close count as connected on the way home
GOAL_MOVED = 0.5 # m, a new plan row only when ARiADNE's waypoint has moved this far
NO_WAYPOINT_HOLD = 20.0 # s exploring without any waypoint message: the planner is gone or stuck, go home
FACE_TRAVEL = 0.3 # m, turn the nose towards the waypoint only while it is at least this far away
LEASH = 1.0 # m, the setpoint waits while it is this far ahead of the drone, so it never runs off without it
AT_HEIGHT = 0.15 # m from fly_z that ends the takeoff
INFLATE = 0.6 # m kept from obstacles in the 2D map on the way to a waypoint: drone radius 0.25 m plus margin


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def dist(a, b):
    return math.sqrt(sum((p - q) ** 2 for p, q in zip(a, b)))


class GridPlanner:
    """Paths on the 2D occupancy grid ARiADNE plans on: known free cells only, obstacles inflated by INFLATE."""

    def __init__(self, msg):
        info = msg.info
        self.res = info.resolution
        self.x0, self.y0 = info.origin.position.x, info.origin.position.y
        grid = np.array(msg.data, dtype=np.int8).reshape(info.height, info.width)
        occupied = grid == 100
        inflated = occupied.copy()
        # a cell is blocked when its centre is closer than INFLATE to the edge of an occupied cell
        reach = (INFLATE + 0.5 * self.res) / self.res
        r = int(math.ceil(reach))
        h, w = grid.shape
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                if dx * dx + dy * dy >= reach * reach:
                    continue
                shifted = np.zeros_like(occupied)
                shifted[max(dy, 0):h + min(dy, 0), max(dx, 0):w + min(dx, 0)] = occupied[max(-dy, 0):h - max(dy, 0), max(-dx, 0):w - max(dx, 0)]
                inflated |= shifted
        self.blocked = inflated | (grid == -1)

    def cell(self, p):
        return int(math.floor((p[1] - self.y0) / self.res)), int(math.floor((p[0] - self.x0) / self.res))

    def centre(self, c):
        return self.x0 + (c[1] + 0.5) * self.res, self.y0 + (c[0] + 0.5) * self.res

    def free(self, c, ends):
        h, w = self.blocked.shape
        return 0 <= c[0] < h and 0 <= c[1] < w and (c in ends or not self.blocked[c])

    def clear(self, a, b, ends):
        steps = max(1, int(dist(a, b) / (0.5 * self.res)))
        return all(self.free(self.cell((a[0] + (b[0] - a[0]) * i / steps, a[1] + (b[1] - a[1]) * i / steps)), ends) for i in range(steps + 1))

    def path(self, start, goal):
        """(x, y) points from start to goal, or None when the inflated map has no way there."""
        s, g = self.cell(start), self.cell(goal)
        ends = (s, g)
        if not self.free(s, ends) or not self.free(g, ends):
            return None
        best = {s: 0.0}
        previous = {}
        queue = [(0.0, s)]
        while queue:
            _, c = heapq.heappop(queue)
            if c == g:
                break
            for dy in (-1, 0, 1):
                for dx in (-1, 0, 1):
                    n = (c[0] + dy, c[1] + dx)
                    if n == c or not self.free(n, ends):
                        continue
                    cost = best[c] + math.hypot(dx, dy)
                    if cost < best.get(n, float('inf')):
                        best[n] = cost
                        previous[n] = c
                        heapq.heappush(queue, (cost + math.hypot(n[0] - g[0], n[1] - g[1]), n))
        if g not in best:
            return None
        cells = [g]
        while cells[-1] != s:
            cells.append(previous[cells[-1]])
        points = [tuple(start[:2])] + [self.centre(c) for c in reversed(cells[1:-1])] + [tuple(goal[:2])]
        # keep only the corners: skip ahead to the farthest point still in a clear straight line
        out = [points[0]]
        i = 0
        while i < len(points) - 1:
            j = len(points) - 1
            while j > i + 1 and not self.clear(points[i], points[j], ends):
                j -= 1
            out.append(points[j])
            i = j
        return out


class Mission:
    def __init__(self):
        self.home = rospy.get_param('~home') # on the pit floor, where the sim spawned the drone
        self.fly_z = rospy.get_param('~fly_z') # the one height ARiADNE explores at
        self.speed = rospy.get_param('autonomous_flight/desired_velocity', 1.0)
        self.yaw_rate = rospy.get_param('autonomous_flight/desired_angular_velocity', 1.0)
        self.stall_seconds = rospy.get_param('~stall_seconds', 120.0)
        self.stall_gain = rospy.get_param('autonomous_flight/complete_min_gain', 0.25)

        self.state = 'IDLE'
        self.position = None
        self.odom_yaw = 0.0
        self.setpoint = list(self.home)
        self.yaw = 0.0
        self.yaw_target = 0.0
        self.route = []
        self.crumbs = []
        self.waypoint = None
        self.waypoint_time = rospy.Time(0)
        self.last_goal = None
        self.plan_time = 0.0
        self.planner_done = False
        self.best_block = 0.0
        self.best_block_time = None
        self.explore_since = None
        self.grid = None
        self.grid_msg = None

        self.target_pub = rospy.Publisher('/autonomous_flight/target_state', Target, queue_size=10)
        # the Gazebo quadcopter ignores acceleration commands until it gets a takeoff message, like DEP sends
        self.takeoff_pub = rospy.Publisher(rospy.get_param('autonomous_flight/takeoff_topic', '/CERLAB/quadcopter/takeoff'), Empty, queue_size=1, latch=True)
        self.land_pub = rospy.Publisher(rospy.get_param('autonomous_flight/land_topic', '/CERLAB/quadcopter/land'), Empty, queue_size=1, latch=True)
        self.state_pub = rospy.Publisher('/ariadne_mission/state', String, queue_size=10, latch=True)
        self.event_pub = rospy.Publisher('/ariadne_mission/event', String, queue_size=50)
        self.plan_pub = rospy.Publisher('/ariadne_mission/plan', String, queue_size=50)
        self.path_pub = rospy.Publisher('/dynamicExploration/bspline_trajectory', Path, queue_size=1)

        rospy.Subscriber('/CERLAB/quadcopter/odom', Odometry, self.odom_cb, queue_size=10)
        rospy.Subscriber('/way_point', PointStamped, self.waypoint_cb, queue_size=10)
        rospy.Subscriber('/runtime', Float32, self.runtime_cb, queue_size=10)
        rospy.Subscriber('/projected_map', OccupancyGrid, self.map_cb, queue_size=1)
        rospy.Subscriber('/exploration/status', String, self.status_cb, queue_size=1)

        rospy.Service('/exploration/start', Trigger, self.start_srv)
        rospy.Service('/exploration/return_home', Trigger, self.return_srv)
        rospy.Service('/exploration/land', Trigger, self.land_srv)
        rospy.Service('/exploration/pause', Trigger, self.no_pause_srv)
        rospy.Service('/exploration/resume', Trigger, self.no_pause_srv)

        self.set_state('IDLE')
        rospy.Timer(rospy.Duration(1.0 / RATE), self.tick)

    def set_state(self, state):
        self.state = state
        self.state_pub.publish(String(state))

    def event(self, text):
        rospy.loginfo('[ariadne_mission] ' + text)
        self.event_pub.publish(String(text))

    # ------------------------------------------------------------------ inputs
    def odom_cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.position = (p.x, p.y, p.z)
        self.odom_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        if self.state not in ('IDLE', 'LANDED') and (not self.crumbs or dist(self.crumbs[-1], self.position) >= CRUMB_SPACING):
            self.crumbs.append(self.position)

    def map_cb(self, msg):
        self.grid_msg = msg # turned into a GridPlanner only when a new waypoint needs a path
        self.grid = None

    def runtime_cb(self, msg):
        # rl_planner publishes its planning time in seconds, and exactly 0 once no frontier has utility left
        if msg.data == 0.0:
            if not self.planner_done and self.state == 'EXPLORING':
                self.event('ARiADNE reports exploration completed')
            self.planner_done = True
        else:
            self.plan_time = msg.data

    def waypoint_cb(self, msg):
        self.waypoint_time = rospy.Time.now()
        goal = (msg.point.x, msg.point.y, self.fly_z)
        if self.state != 'EXPLORING' or self.position is None:
            return
        self.waypoint = goal
        if self.last_goal is not None and dist(goal, self.last_goal) < GOAL_MOVED:
            return
        self.last_goal = goal
        if self.grid is None and self.grid_msg is not None:
            self.grid = GridPlanner(self.grid_msg)
        points = self.grid.path(self.setpoint, goal) if self.grid is not None else None
        if points is None:
            self.event('No clear path to ARiADNE waypoint (%.1f, %.1f) in its own map: flying straight' % goal[:2])
            points = [tuple(self.setpoint[:2]), goal[:2]]
        route = [(x, y, self.fly_z) for x, y in points[1:]]
        self.route = route
        length = sum(dist(a, b) for a, b in zip([tuple(self.setpoint)] + route, route))
        self.plan_pub.publish(String(json.dumps({'goal': goal, 'from': self.position, 'path_length': length, 'legs': len(route),
                                                 'path_time': length / self.speed, 'plan_time': self.plan_time})))
        path = Path()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = 'map'
        for p in [tuple(self.setpoint)] + route:
            pose = PoseStamped()
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = p
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self.path_pub.publish(path)

    def status_cb(self, msg):
        if self.state != 'EXPLORING':
            return
        block = json.loads(msg.data).get('explored_block_pct', 0.0)
        if self.best_block_time is None or block >= self.best_block + self.stall_gain:
            self.best_block = block
            self.best_block_time = rospy.Time.now()

    # ------------------------------------------------------------------ dashboard buttons
    def start_srv(self, _):
        if self.state != 'IDLE':
            return TriggerResponse(False, 'Mission already started (%s)' % self.state)
        if self.position is None:
            return TriggerResponse(False, 'No odometry yet')
        self.setpoint = list(self.position)
        self.yaw = self.yaw_target = self.odom_yaw
        self.crumbs = [self.position]
        self.route = [(self.setpoint[0], self.setpoint[1], self.fly_z)]
        self.takeoff_pub.publish(Empty())
        self.set_state('TAKEOFF')
        self.event('Taking off from the pit floor to z %.1f m' % self.fly_z)
        return TriggerResponse(True, 'Mission started')

    def return_srv(self, _):
        if self.state != 'EXPLORING':
            return TriggerResponse(False, 'Cannot return home now (%s)' % self.state)
        self.start_return('Return home requested')
        return TriggerResponse(True, 'Returning home')

    def land_srv(self, _):
        if self.state not in ('TAKEOFF', 'EXPLORING', 'RETURNING'):
            return TriggerResponse(False, 'Cannot land now (%s)' % self.state)
        self.setpoint = list(self.position)
        self.start_landing('Landing where it is: the ground below is not checked')
        return TriggerResponse(True, 'Landing')

    def no_pause_srv(self, _):
        return TriggerResponse(False, 'Pause is not available with ARiADNE')

    # ------------------------------------------------------------------ flight
    def start_return(self, reason):
        self.event(reason + ': retracing the flown path')
        self.route = self.home_route()
        self.setpoint = list(self.position)
        self.set_state('RETURNING')

    def start_landing(self, reason):
        self.event(reason)
        self.route = [(self.setpoint[0], self.setpoint[1], self.home[2])]
        self.set_state('LANDING')

    def home_route(self):
        """Shortest way back over the takeoff spot through remembered positions, so it only uses space already flown."""
        crumbs = self.crumbs + [self.position]
        goal = (self.home[0], self.home[1], self.fly_z)
        goal_index = min(range(len(crumbs)), key=lambda i: dist(crumbs[i], goal))
        cells = {}
        for i, c in enumerate(crumbs):
            cells.setdefault(tuple(int(math.floor(v)) for v in c), []).append(i)

        def neighbours(i):
            cx, cy, cz = (int(math.floor(v)) for v in crumbs[i])
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    for dz in (-1, 0, 1):
                        for j in cells.get((cx + dx, cy + dy, cz + dz), ()):
                            d = dist(crumbs[i], crumbs[j])
                            # consecutive ones were flown directly, however far apart
                            if j != i and (d <= CRUMB_LINK or abs(i - j) == 1):
                                yield j, d

        start = len(crumbs) - 1
        best = {start: 0.0}
        previous = {}
        queue = [(0.0, start)]
        while queue:
            cost, i = heapq.heappop(queue)
            if i == goal_index:
                break
            if cost > best[i]:
                continue
            for j, d in neighbours(i):
                if cost + d < best.get(j, float('inf')):
                    best[j] = cost + d
                    previous[j] = i
                    heapq.heappush(queue, (cost + d, j))
        route = [goal_index]
        while route[-1] in previous:
            route.append(previous[route[-1]])
        route.reverse()
        return [crumbs[i] for i in route] + [goal]

    def follow_route(self, speed, dt):
        """Moves the setpoint along the route. Returns its velocity, or None once the route is done."""
        if not self.route:
            return None
        if self.position is not None and dist(self.setpoint, self.position) > LEASH:
            return [0.0, 0.0, 0.0]
        left = speed * dt
        while self.route:
            d = dist(self.setpoint, self.route[0])
            if d > left:
                direction = [(q - p) / d for p, q in zip(self.setpoint, self.route[0])]
                self.setpoint = [p + left * v for p, v in zip(self.setpoint, direction)]
                return [speed * v for v in direction]
            self.setpoint = list(self.route.pop(0))
            left -= d
        return [0.0, 0.0, 0.0]

    def send(self, velocity):
        step = max(-self.yaw_rate / RATE, min(self.yaw_rate / RATE, wrap(self.yaw_target - self.yaw)))
        self.yaw = wrap(self.yaw + step)
        target = Target()
        target.header.stamp = rospy.Time.now()
        target.header.frame_id = 'map'
        target.type_mask = Target.IGNORE_ACC # position and velocity reference, no feedforward acceleration
        target.position.x, target.position.y, target.position.z = self.setpoint
        target.velocity.x, target.velocity.y, target.velocity.z = velocity
        target.yaw = self.yaw
        self.target_pub.publish(target)

    def tick(self, _):
        if self.state == 'IDLE':
            return # nothing sent: the controller stays off until the mission starts
        dt = 1.0 / RATE
        velocity = [0.0, 0.0, 0.0]
        if self.state == 'TAKEOFF':
            velocity = self.follow_route(0.5, dt)
            if not self.route and abs(self.position[2] - self.fly_z) < AT_HEIGHT:
                self.event('At z %.1f m: following ARiADNE' % self.fly_z)
                self.explore_since = rospy.Time.now()
                self.best_block_time = None
                self.set_state('EXPLORING')
        elif self.state == 'EXPLORING':
            last_waypoint = max(self.waypoint_time, self.explore_since)
            if self.planner_done:
                self.start_return('ARiADNE finished: no frontier with utility left')
            elif (rospy.Time.now() - last_waypoint).to_sec() >= NO_WAYPOINT_HOLD:
                self.start_return('ARiADNE sent no waypoint for %.0f s' % NO_WAYPOINT_HOLD)
            elif self.best_block_time is not None and (rospy.Time.now() - self.best_block_time).to_sec() > self.stall_seconds:
                self.start_return('Coverage gained under %.2f points in %.0f s' % (self.stall_gain, self.stall_seconds))
            elif self.route:
                if dist(self.setpoint, self.route[0]) > FACE_TRAVEL:
                    self.yaw_target = math.atan2(self.route[0][1] - self.setpoint[1], self.route[0][0] - self.setpoint[0])
                velocity = self.follow_route(self.speed, dt)
        elif self.state == 'RETURNING':
            if self.route and dist(self.setpoint, self.route[0]) > 0.1:
                self.yaw_target = math.atan2(self.route[0][1] - self.setpoint[1], self.route[0][0] - self.setpoint[0])
            velocity = self.follow_route(self.speed, dt)
            if not self.route:
                self.start_landing('Over the takeoff spot: landing')
        elif self.state == 'LANDING':
            velocity = self.follow_route(0.3, dt)
            if not self.route:
                self.land_pub.publish(Empty())
                self.event('Landed')
                self.set_state('LANDED')
        self.send(velocity or [0.0, 0.0, 0.0])


if __name__ == '__main__':
    rospy.init_node('ariadne_mission')
    Mission()
    rospy.spin()
