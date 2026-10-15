#!/usr/bin/env python3
"""Per-goal efficiency metrics for Nav2 and DQN, written to <results_dir>/runs.csv.

An "episode" starts and ends on one of:
  * RViz "Nav2 Goal" click: /goal_pose starts it; the navigate_to_pose action
    reaching SUCCEEDED / ABORTED / CANCELED ends it.
  * /tb3_maze/episode_event (std_msgs/String, JSON) from goal_runner or
    maze_dqn_gazebo: {"event": "start", "label": .., "goal": [x, y]} and
    {"event": "end", "result": "succeeded" | "failed" | ...}.

Metrics per episode (identical for both methods):
  time_s          sim-time from start to end
  driven_m        path actually driven (integrated /odom positions)
  planned_m       length of the first /plan after start (Nav2 only)
  straight_m      straight-line start -> goal
  efficiency      straight_m / driven_m  (1.0 = perfectly straight)
  min_clear_m     minimum valid LiDAR range seen (closest approach to a wall)
  near_collisions times the LiDAR minimum dropped below `collision_dist`
                  (0.15 m, the same threshold the DQN environment uses)
  rotation_rad    total absolute heading change (smoothness: lower = smoother)
"""

import csv
from datetime import datetime
import json
import math
import os

from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

FIELDS = ['run_id', 'wall_time', 'method', 'label', 'result', 'goal_x', 'goal_y',
          'start_x', 'start_y', 'time_s', 'driven_m', 'planned_m', 'straight_m',
          'efficiency', 'min_clear_m', 'near_collisions', 'rotation_rad']

TERMINAL = {GoalStatus.STATUS_SUCCEEDED: 'succeeded',
            GoalStatus.STATUS_ABORTED: 'aborted',
            GoalStatus.STATUS_CANCELED: 'canceled'}


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def path_length(poses):
    total = 0.0
    for a, b in zip(poses, poses[1:]):
        total += math.hypot(b.pose.position.x - a.pose.position.x,
                            b.pose.position.y - a.pose.position.y)
    return total


class Episode:

    def __init__(self, label, goal, start_xy, t0, source):
        self.label = label
        self.goal = goal
        self.start = start_xy
        self.t0 = t0
        self.source = source            # 'click' or 'event'
        self.driven = 0.0
        self.rotation = 0.0
        self.planned = None
        self.min_clear = float('inf')
        self.near_collisions = 0
        self.below = False
        self.last_xy = start_xy
        self.last_yaw = None
        self.goal_ids = set()


class MetricsLogger(Node):

    def __init__(self):
        super().__init__('metrics_logger')
        self.declare_parameter('method', 'nav2')
        self.declare_parameter('results_dir', os.path.expanduser('~/tb3_maze_results'))
        self.declare_parameter('collision_dist', 0.15)
        self.declare_parameter('action_name', 'navigate_to_pose')

        self.method = self.get_parameter('method').value
        self.collision_dist = float(self.get_parameter('collision_dist').value)
        results_dir = os.path.expanduser(self.get_parameter('results_dir').value)
        os.makedirs(results_dir, exist_ok=True)
        self.csv_path = os.path.join(results_dir, 'runs.csv')
        if not os.path.exists(self.csv_path):
            with open(self.csv_path, 'w', newline='') as f:
                csv.writer(f).writerow(FIELDS)

        self.ep = None
        self.pose = None            # latest odom (x, y)
        self.seen_ids = set()       # action goal ids already attributed to an episode

        status_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10,
                                reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        action = self.get_parameter('action_name').value
        self.create_subscription(Odometry, 'odom', self.on_odom, 20)
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Path, 'plan', self.on_plan, 10)
        self.create_subscription(PoseStamped, 'goal_pose', self.on_click, 10)
        self.create_subscription(GoalStatusArray, f'{action}/_action/status',
                                 self.on_status, status_qos)
        self.create_subscription(String, '/tb3_maze/episode_event', self.on_event, 10)
        self.get_logger().info(f'metrics_logger: method="{self.method}" -> {self.csv_path}')

    # ------------------------------------------------------------------ utils
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def start(self, label, goal, source):
        if self.ep is not None:
            self.finish('preempted')
        start_xy = self.pose or (0.0, 0.0)
        self.ep = Episode(label, goal, start_xy, self.now(), source)
        self.get_logger().info(f'episode start: {label} goal=({goal[0]:.2f}, {goal[1]:.2f})')

    def finish(self, result):
        ep, self.ep = self.ep, None
        if ep is None:
            return
        dt = self.now() - ep.t0
        straight = math.hypot(ep.goal[0] - ep.start[0], ep.goal[1] - ep.start[1])
        eff = straight / ep.driven if ep.driven > 1e-3 else 0.0
        with open(self.csv_path, newline='') as f:
            run_id = sum(1 for _ in f)          # header counts as 1 -> ids start at 1
        row = {
            'run_id': run_id,
            'wall_time': datetime.now().isoformat(timespec='seconds'),
            'method': self.method, 'label': ep.label, 'result': result,
            'goal_x': f'{ep.goal[0]:.3f}', 'goal_y': f'{ep.goal[1]:.3f}',
            'start_x': f'{ep.start[0]:.3f}', 'start_y': f'{ep.start[1]:.3f}',
            'time_s': f'{dt:.2f}', 'driven_m': f'{ep.driven:.3f}',
            'planned_m': '' if ep.planned is None else f'{ep.planned:.3f}',
            'straight_m': f'{straight:.3f}', 'efficiency': f'{eff:.3f}',
            'min_clear_m': '' if math.isinf(ep.min_clear) else f'{ep.min_clear:.3f}',
            'near_collisions': ep.near_collisions,
            'rotation_rad': f'{ep.rotation:.2f}',
        }
        with open(self.csv_path, 'a', newline='') as f:
            csv.DictWriter(f, fieldnames=FIELDS).writerow(row)
        self.get_logger().info(
            f'episode end: {ep.label} {result} | {dt:.1f} s, driven {ep.driven:.2f} m, '
            f'planned {row["planned_m"] or "-"} m, eff {eff:.2f}, '
            f'min clear {row["min_clear_m"] or "-"} m, near-collisions {ep.near_collisions}')

    # -------------------------------------------------------------- callbacks
    def on_odom(self, msg):
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        yaw = yaw_of(msg.pose.pose.orientation)
        # A respawned robot (DQN reset) restarts odom at 0: treat a jump as a reset.
        if self.pose is not None and math.hypot(x - self.pose[0], y - self.pose[1]) > 0.5:
            if self.ep is not None:
                self.ep.last_xy, self.ep.last_yaw = (x, y), yaw
                if self.ep.driven < 0.05:      # reset landed just after the start event
                    self.ep.start = (x, y)
        self.pose = (x, y)
        ep = self.ep
        if ep is None:
            return
        step = math.hypot(x - ep.last_xy[0], y - ep.last_xy[1])
        if step > 0.001:
            ep.driven += step
            ep.last_xy = (x, y)
        if ep.last_yaw is not None:
            d = math.atan2(math.sin(yaw - ep.last_yaw), math.cos(yaw - ep.last_yaw))
            ep.rotation += abs(d)
        ep.last_yaw = yaw

    def on_scan(self, msg):
        ep = self.ep
        if ep is None:
            return
        valid = [r for r in msg.ranges if msg.range_min <= r <= msg.range_max]
        if not valid:
            return
        m = min(valid)
        ep.min_clear = min(ep.min_clear, m)
        if m < self.collision_dist and not ep.below:
            ep.near_collisions += 1
        ep.below = m < self.collision_dist

    def on_plan(self, msg):
        if self.ep is not None and self.ep.planned is None and len(msg.poses) > 1:
            self.ep.planned = path_length(msg.poses)

    def on_click(self, msg):
        # RViz "Nav2 Goal". goal_runner sends actions directly, so it never lands here.
        self._click_count = getattr(self, '_click_count', 0) + 1
        p = msg.pose.position
        self.start(f'click{self._click_count}', (p.x, p.y), 'click')

    def on_status(self, msg):
        ep = self.ep
        if ep is None or ep.source != 'click':
            return
        for st in msg.status_list:
            gid = bytes(st.goal_info.goal_id.uuid)
            if st.status in (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING):
                if gid not in self.seen_ids:
                    self.seen_ids.add(gid)
                    ep.goal_ids.add(gid)
            elif st.status in TERMINAL and gid in ep.goal_ids:
                self.finish(TERMINAL[st.status])
                return

    def on_event(self, msg):
        try:
            ev = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f'bad episode_event: {msg.data}')
            return
        if ev.get('event') == 'start':
            g = ev.get('goal', [0.0, 0.0])
            self.start(ev.get('label', 'goal'), (float(g[0]), float(g[1])), 'event')
            if ev.get('method'):
                self.method = ev['method']
        elif ev.get('event') == 'end' and self.ep is not None:
            self.finish(ev.get('result', 'unknown'))


def main(args=None):
    rclpy.init(args=args)
    node = MetricsLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ep is not None:
            node.finish('interrupted')
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
