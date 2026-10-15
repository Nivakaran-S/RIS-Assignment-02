#!/usr/bin/env python3
"""Maze-aware drop-in replacement for `turtlebot3_dqn dqn_gazebo`.

Serves the same three services the official dqn_environment calls
(turtlebot3_msgs/srv/Goal): initialize_env, task_succeed, task_failed.

Differences from the official node (ROBOTIS turtlebot3_machine_learning, jazzy):
  * goals are free cell centres of OUR maze, never inside a wall
  * curriculum: goals start within `start_hops` cell moves of the robot and the
    radius grows by one every `promote_after` successes
  * eval_mode: cycles the fixed goals in config/goals.yaml and always respawns
    the robot at the origin, so DQN is measured on exactly the Nav2 trials
  * publishes /tb3_maze/episode_event (JSON) so metrics_logger can score DQN
    episodes with the same metrics as Nav2

Gazebo entity create/remove uses the `gz service` CLI, like the official node.
"""

import json
import os
import random
import subprocess
import time

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from std_msgs.msg import String
import yaml

from tb3_maze.maze_geometry import MazeLayout, default_layout_path
from turtlebot3_msgs.srv import Goal


class MazeDqnGazebo(Node):

    def __init__(self):
        super().__init__('gazebo_interface')
        self.declare_parameter('world_name', 'dqn')
        self.declare_parameter('robot_name', 'burger')
        self.declare_parameter('robot_sdf', '')
        self.declare_parameter('goal_sdf', '')
        self.declare_parameter('layout_file', '')
        self.declare_parameter('goals_file', '')
        self.declare_parameter('eval_mode', False)
        self.declare_parameter('start_hops', 2)
        self.declare_parameter('promote_after', 5)
        self.declare_parameter('seed', 0)

        gp = self.get_parameter
        self.world = gp('world_name').value
        self.robot_name = gp('robot_name').value
        self.robot_sdf = gp('robot_sdf').value or os.path.join(
            get_package_share_directory('turtlebot3_gazebo'),
            'models', 'turtlebot3_burger', 'model.sdf')
        self.goal_sdf = gp('goal_sdf').value or os.path.join(
            get_package_share_directory('turtlebot3_gazebo'),
            'models', 'turtlebot3_dqn_world', 'goal_box', 'model.sdf')
        layout_file = gp('layout_file').value or str(default_layout_path())
        goals_file = gp('goals_file').value or os.path.join(
            os.path.dirname(layout_file), 'goals.yaml')
        self.eval_mode = bool(gp('eval_mode').value)
        self.start_hops = int(gp('start_hops').value)
        self.promote_after = max(1, int(gp('promote_after').value))
        if gp('seed').value:
            random.seed(int(gp('seed').value))

        self.layout = MazeLayout.load(layout_file)
        self.origin_cell = (self.layout.origin_col, self.layout.origin_row)
        self.robot_cell = self.origin_cell
        self.successes = 0
        self.max_hops_possible = max(self.layout.bfs_hops().values())

        with open(goals_file) as f:
            self.fixed_goals = yaml.safe_load(f)['goals']
        self.eval_index = 0
        self.goal_label = ''
        self.goal_x, self.goal_y = self._next_goal()

        self.event_pub = self.create_publisher(String, '/tb3_maze/episode_event', 10)
        group = MutuallyExclusiveCallbackGroup()
        self.create_service(Goal, 'initialize_env', self.initialize_env_cb, callback_group=group)
        self.create_service(Goal, 'task_succeed', self.task_succeed_cb, callback_group=group)
        self.create_service(Goal, 'task_failed', self.task_failed_cb, callback_group=group)

        mode = 'EVAL (fixed goals G1..G3)' if self.eval_mode else \
            f'TRAIN (curriculum from {self.start_hops} hops, +1 every {self.promote_after} wins)'
        self.get_logger().info(f'maze_dqn_gazebo ready: world "{self.world}", mode {mode}')

    # ------------------------------------------------------------- goal logic
    def current_max_hops(self) -> int:
        return min(self.max_hops_possible,
                   self.start_hops + self.successes // self.promote_after)

    def _next_goal(self):
        if self.eval_mode:
            g = self.fixed_goals[self.eval_index % len(self.fixed_goals)]
            self.eval_index += 1
            self.goal_label = g['name']
            return float(g['x']), float(g['y'])
        hops = self.layout.bfs_hops(self.robot_cell)
        limit = self.current_max_hops()
        candidates = [c for c, h in hops.items() if 1 <= h <= limit]
        cell = random.choice(candidates)
        self.goal_label = f'cell{cell[0]}{cell[1]}_h{hops[cell]}'
        return self.layout.cell_center(*cell)

    # ------------------------------------------------------- gazebo services
    def _gz(self, service, reqtype, req) -> bool:
        cmd = ['gz', 'service', '-s', f'/world/{self.world}/{service}',
               '--reqtype', reqtype, '--reptype', 'gz.msgs.Boolean',
               '--timeout', '2000', '--req', req]
        try:
            subprocess.run(cmd, check=True, stdout=subprocess.DEVNULL,
                           stderr=subprocess.DEVNULL, timeout=5)
            return True
        except (subprocess.CalledProcessError, subprocess.TimeoutExpired):
            self.get_logger().warn(f'gz service {service} failed')
            return False

    def _remove(self, name):
        self._gz('remove', 'gz.msgs.Entity', f'name: "{name}", type: 2')

    def _spawn(self, sdf_path, name, x, y):
        self._gz('create', 'gz.msgs.EntityFactory',
                 f'sdf_filename: "{sdf_path}", name: "{name}", '
                 f'pose: {{ position: {{ x: {x}, y: {y}, z: 0.0 }} }}')

    def respawn_goal(self):
        self._remove('goal_box')
        time.sleep(0.2)
        self._spawn(self.goal_sdf, 'goal_box', self.goal_x, self.goal_y)

    def reset_robot(self):
        # Respawn at the origin: odom restarts at 0, so odom == world == map,
        # which dqn_environment relies on to compute goal distance/angle.
        self._remove(self.robot_name)
        time.sleep(0.3)
        self._spawn(self.robot_sdf, self.robot_name, 0.0, 0.0)
        self.robot_cell = self.origin_cell

    def _event(self, **kw):
        self.event_pub.publish(String(data=json.dumps(kw)))

    def _start_episode(self, response):
        self.respawn_goal()
        self._event(event='start', label=self.goal_label, goal=[self.goal_x, self.goal_y],
                    max_hops=self.current_max_hops())
        response.pose_x = float(self.goal_x)
        response.pose_y = float(self.goal_y)
        response.success = True
        self.get_logger().info(
            f'goal {self.goal_label} at ({self.goal_x:.2f}, {self.goal_y:.2f}); '
            f'successes={self.successes} max_hops={self.current_max_hops()}')
        return response

    # ------------------------------------------------------------- callbacks
    def initialize_env_cb(self, request, response):
        self.reset_robot()
        time.sleep(0.2)
        return self._start_episode(response)

    def task_succeed_cb(self, request, response):
        self._event(event='end', result='succeeded')
        self.successes += 1
        goal_cell = self.layout.world_to_cell(self.goal_x, self.goal_y)
        if self.eval_mode:
            self.reset_robot()
        elif goal_cell:
            self.robot_cell = goal_cell
        self.goal_x, self.goal_y = self._next_goal()
        time.sleep(0.2)
        return self._start_episode(response)

    def task_failed_cb(self, request, response):
        self._event(event='end', result='failed')
        self.reset_robot()
        self.goal_x, self.goal_y = self._next_goal()
        time.sleep(0.2)
        return self._start_episode(response)


def main(args=None):
    rclpy.init(args=args)
    node = MazeDqnGazebo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
