"""VIVA demo: one command brings up everything needed to click goals.

    ros2 launch tb3_maze demo.launch.py [planner:=astar]

Gazebo maze + Burger, Nav2 with the saved SLAM map, RViz (Nav2 Goal tool),
initial pose pre-set, and metrics_logger recording every clicked goal to
~/tb3_maze_results/runs.csv.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg = get_package_share_directory('tb3_maze')
    return LaunchDescription([
        DeclareLaunchArgument('planner', default_value='dijkstra'),
        DeclareLaunchArgument('map', default_value=os.path.join(pkg, 'maps', 'maze_map.yaml')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'navigation.launch.py')),
            launch_arguments={
                'sim': 'true', 'rviz': 'true', 'metrics': 'true',
                'planner': LaunchConfiguration('planner'),
                'map': LaunchConfiguration('map'),
            }.items()),
    ])
