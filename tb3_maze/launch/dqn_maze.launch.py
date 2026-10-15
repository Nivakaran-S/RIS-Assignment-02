"""DQN training/evaluation inside OUR maze (instead of the ROBOTIS stage worlds).

    ros2 launch tb3_maze dqn_maze.launch.py                 # terminal 1
    ros2 run turtlebot3_dqn dqn_environment                 # terminal 2 (official, unchanged)
    ros2 run turtlebot3_dqn dqn_agent --ros-args \\
        -p max_training_episodes:=300 -p epsilon_decay:=2500  # terminal 3 (official)

What this launch replaces / adds:
  * world   : worlds/maze_dqn.world (same maze, world named "dqn")
  * robot   : a copy of the Burger with a 48-ray LiDAR. The official agent hard-codes
              state_size = 26 = 2 (goal dist, angle) + 24 front rays; the stock
              360-ray Burger would give ~181 front rays and crash the agent.
              The SLAM/Nav2 robot is NOT modified.
  * goals   : tb3_maze maze_dqn_gazebo replaces `turtlebot3_dqn dqn_gazebo`. It serves
              the same services (initialize_env / task_succeed / task_failed) but
              samples goals at free cell centres with a hop-distance curriculum,
              instead of random points in a 4.2 x 4.2 m square.

Args: eval:=true uses the fixed goals in config/goals.yaml (for the Nav2 vs DQN table).
"""

import os
import re

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

DQN_LIDAR_SAMPLES = 48


def make_dqn_robot_sdf() -> str:
    """Copy the official Burger SDF with the LiDAR reduced to 48 samples."""
    src = os.path.join(get_package_share_directory('turtlebot3_gazebo'),
                       'models', 'turtlebot3_burger', 'model.sdf')
    with open(src) as f:
        sdf = f.read()
    sdf, n = re.subn(r'<samples>\s*360\s*</samples>',
                     f'<samples>{DQN_LIDAR_SAMPLES}</samples>', sdf)
    if n != 1:
        raise RuntimeError(f'Expected one 360-sample LiDAR in {src}, found {n}')
    out_dir = os.path.join(os.path.expanduser('~'), '.ros', 'tb3_maze', 'turtlebot3_burger_dqn')
    os.makedirs(out_dir, exist_ok=True)
    out = os.path.join(out_dir, 'model.sdf')
    with open(out, 'w') as f:
        f.write(sdf)
    return out


def _setup(context):
    pkg = get_package_share_directory('tb3_maze')
    robot_sdf = make_dqn_robot_sdf()
    evaluate = LaunchConfiguration('eval').perform(context).lower() == 'true'

    maze = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'maze.launch.py')),
        launch_arguments={
            'world': os.path.join(pkg, 'worlds', 'maze_dqn.world'),
            'robot_sdf': robot_sdf,
            'robot_name': 'burger',
            'headless': LaunchConfiguration('headless').perform(context),
        }.items())

    gazebo_interface = Node(
        package='tb3_maze', executable='maze_dqn_gazebo', output='screen',
        parameters=[{
            'use_sim_time': True,
            'world_name': 'dqn',
            'robot_name': 'burger',
            'robot_sdf': robot_sdf,
            'eval_mode': evaluate,
            'start_hops': int(LaunchConfiguration('start_hops').perform(context)),
            'promote_after': int(LaunchConfiguration('promote_after').perform(context)),
        }])

    actions = [maze, gazebo_interface]
    if evaluate:
        actions.append(Node(
            package='tb3_maze', executable='metrics_logger', output='screen',
            parameters=[{'use_sim_time': True, 'method': 'dqn',
                         'results_dir': os.path.expanduser('~/tb3_maze_results')}]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('eval', default_value='false',
                              description='true = fixed goals G1..G3 + metrics_logger'),
        DeclareLaunchArgument('start_hops', default_value='2',
                              description='curriculum: max BFS moves to the first goals'),
        DeclareLaunchArgument('promote_after', default_value='5',
                              description='curriculum: successes before allowing +1 hop'),
        OpaqueFunction(function=_setup),
    ])
