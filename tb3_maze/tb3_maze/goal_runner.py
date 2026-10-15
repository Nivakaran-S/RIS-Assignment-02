#!/usr/bin/env python3
"""Repeatable Nav2 trials: drive to each goal in config/goals.yaml N times.

    ros2 run tb3_maze goal_runner --ros-args -p repeats:=3 -p method:=nav2_astar

Requires Nav2 running (navigation.launch.py) and metrics_logger running (it is,
by default, in navigation.launch.py). For each trial:
  1. (untracked) return to the start pose (0,0,0) so every trial is start -> goal
  2. publish an episode "start" event, send the goal, wait for the result
  3. publish an episode "end" event with the result
"""

import json
import math
import os
import time

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String
import yaml

RESULT = {GoalStatus.STATUS_SUCCEEDED: 'succeeded',
          GoalStatus.STATUS_ABORTED: 'aborted',
          GoalStatus.STATUS_CANCELED: 'canceled'}


class GoalRunner(Node):

    def __init__(self):
        super().__init__('goal_runner')
        self.declare_parameter('goals_file', os.path.join(
            get_package_share_directory('tb3_maze'), 'config', 'goals.yaml'))
        self.declare_parameter('repeats', 3)
        self.declare_parameter('method', '')
        self.declare_parameter('return_to_start', True)
        self.declare_parameter('timeout_s', 120.0)
        self.declare_parameter('only', '')     # e.g. "G2" to run a single goal

        with open(self.get_parameter('goals_file').value) as f:
            self.goals = yaml.safe_load(f)['goals']
        only = self.get_parameter('only').value
        if only:
            self.goals = [g for g in self.goals if g['name'] == only]
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.events = self.create_publisher(String, '/tb3_maze/episode_event', 10)

    def pose(self, x, y, yaw):
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x = float(x)
        p.pose.position.y = float(y)
        p.pose.orientation.z = math.sin(yaw / 2.0)
        p.pose.orientation.w = math.cos(yaw / 2.0)
        return p

    def navigate(self, x, y, yaw):
        goal = NavigateToPose.Goal()
        goal.pose = self.pose(x, y, yaw)
        send = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send)
        handle = send.result()
        if handle is None or not handle.accepted:
            return 'rejected'
        res = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res,
                                         timeout_sec=self.get_parameter('timeout_s').value)
        if not res.done():
            handle.cancel_goal_async()
            return 'timeout'
        return RESULT.get(res.result().status, f'status{res.result().status}')

    def event(self, **kw):
        self.events.publish(String(data=json.dumps(kw)))
        # give metrics_logger a moment to receive it before motion starts
        end = time.time() + 0.3
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def run(self):
        self.get_logger().info('waiting for navigate_to_pose action server...')
        self.client.wait_for_server()
        repeats = int(self.get_parameter('repeats').value)
        method = self.get_parameter('method').value
        summary = []
        for rep in range(1, repeats + 1):
            for g in self.goals:
                if self.get_parameter('return_to_start').value:
                    r = self.navigate(0.0, 0.0, 0.0)
                    if r != 'succeeded':
                        self.get_logger().warn(f'return to start: {r}')
                label = f'{g["name"]}_r{rep}'
                start = {'event': 'start', 'label': label, 'goal': [g['x'], g['y']]}
                if method:
                    start['method'] = method
                self.event(**start)
                t0 = time.time()
                result = self.navigate(g['x'], g['y'], g.get('yaw', 0.0))
                self.event(event='end', result=result)
                summary.append((label, result, time.time() - t0))
                self.get_logger().info(f'{label}: {result} ({summary[-1][2]:.1f} s wall)')
        ok = sum(1 for _, r, _ in summary if r == 'succeeded')
        self.get_logger().info(f'done: {ok}/{len(summary)} succeeded. See runs.csv.')


def main(args=None):
    rclpy.init(args=args)
    node = GoalRunner()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
