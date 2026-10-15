"""Gazebo Harmonic + maze world + TurtleBot3 Burger + ROS<->GZ bridge.

    ros2 launch tb3_maze maze.launch.py [headless:=true] [x_pose:=0.0 y_pose:=0.0 yaw:=0.0]

Every other launch file in this package includes this one, so the world,
spawn pose and use_sim_time are defined in exactly one place.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription)
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # The official TurtleBot3 launch includes read this at load time.
    os.environ.setdefault('TURTLEBOT3_MODEL', 'burger')
    model = os.environ['TURTLEBOT3_MODEL']

    pkg = get_package_share_directory('tb3_maze')
    tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')

    world = LaunchConfiguration('world')
    headless = LaunchConfiguration('headless')
    use_sim_time = LaunchConfiguration('use_sim_time')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    yaw = LaunchConfiguration('yaw')
    robot_sdf = LaunchConfiguration('robot_sdf')
    robot_name = LaunchConfiguration('robot_name')
    gz_verbosity = LaunchConfiguration('gz_verbosity')

    declare = [
        DeclareLaunchArgument('world', default_value=os.path.join(pkg, 'worlds', 'maze.world'),
                              description='Absolute path to the SDF world'),
        DeclareLaunchArgument('headless', default_value='false',
                              description='true = run gz server only (no Gazebo GUI)'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('x_pose', default_value='0.0'),
        DeclareLaunchArgument('y_pose', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument(
            'robot_sdf',
            default_value=os.path.join(tb3_gazebo, 'models', f'turtlebot3_{model}', 'model.sdf'),
            description='Robot model to spawn (DQN uses a 48-ray LiDAR variant)'),
        DeclareLaunchArgument('robot_name', default_value=model),
        DeclareLaunchArgument('gz_verbosity', default_value='2'),
    ]

    # Meshes referenced as model://turtlebot3_common/... live here.
    resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', os.path.join(tb3_gazebo, 'models'))

    gz_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': ['-r -s -v', gz_verbosity, ' ', world],
            'on_exit_shutdown': 'true',
        }.items())

    gz_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': ['-g -v', gz_verbosity], 'on_exit_shutdown': 'true'}.items(),
        condition=UnlessCondition(headless))

    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_gazebo, 'launch', 'robot_state_publisher.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items())

    # Same as turtlebot3_gazebo/spawn_turtlebot3.launch.py, plus yaw and a
    # selectable model file.
    spawn = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-name', robot_name, '-file', robot_sdf,
                   '-x', x_pose, '-y', y_pose, '-z', '0.01', '-Y', yaw])

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        parameters=[{'config_file': os.path.join(tb3_gazebo, 'params',
                                                 f'turtlebot3_{model}_bridge.yaml')}])

    return LaunchDescription(declare + [
        resources, gz_server, gz_client, robot_state_publisher, spawn, bridge])
