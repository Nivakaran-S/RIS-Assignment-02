"""Maze + slam_toolbox (online async) + RViz.

    ros2 launch tb3_maze slam.launch.py                # sim + SLAM + RViz
    ros2 run turtlebot3_teleop teleop_keyboard         # second terminal: drive slowly
    ros2 run nav2_map_server map_saver_cli -f <pkg_src>/maps/maze_map   # save

Args: sim (start Gazebo too, default true), rviz, headless, slam_params_file.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('tb3_maze')
    slam_toolbox = get_package_share_directory('slam_toolbox')

    rviz_cfg = os.path.join(get_package_share_directory('slam_toolbox'),
                            'config', 'slam_toolbox_default.rviz')

    sim = LaunchConfiguration('sim')
    rviz = LaunchConfiguration('rviz')
    headless = LaunchConfiguration('headless')
    params = LaunchConfiguration('slam_params_file')

    return LaunchDescription([
        DeclareLaunchArgument('sim', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('slam_params_file',
                              default_value=os.path.join(pkg, 'config', 'slam_toolbox_maze.yaml')),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'maze.launch.py')),
            launch_arguments={'headless': headless}.items(),
            condition=IfCondition(sim)),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam_toolbox, 'launch', 'online_async_launch.py')),
            launch_arguments={'use_sim_time': 'true', 'slam_params_file': params}.items()),

        Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
             arguments=['-d', rviz_cfg], parameters=[{'use_sim_time': True}],
             condition=IfCondition(rviz)),
    ])
