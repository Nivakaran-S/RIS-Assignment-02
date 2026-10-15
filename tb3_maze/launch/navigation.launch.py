"""Maze + Nav2 (AMCL on a saved map) + RViz click-to-goal.

    ros2 launch tb3_maze navigation.launch.py                       # NavFn Dijkstra
    ros2 launch tb3_maze navigation.launch.py planner:=astar        # NavFn A*
    ros2 launch tb3_maze navigation.launch.py planner:=smac2d       # Smac 2D (A* variant)
    ros2 launch tb3_maze navigation.launch.py max_vel:=0.26 inflation_radius:=0.20

In RViz use "Nav2 Goal" to click a destination. The initial pose is set
automatically (robot spawns at the map origin), so "2D Pose Estimate" is only
needed if AMCL ever loses track.

Map: `map:=<yaml>`. Default is maps/maze_map.yaml (the SLAM map); if that
doesn't exist yet it falls back to maps/maze_groundtruth.yaml with a warning,
so Nav2 tuning can start before mapping is finished.
"""

import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, LogInfo,
                            OpaqueFunction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml

PLANNERS = {
    # NavFn: wavefront Dijkstra over the costmap
    'dijkstra': {'plugin': 'nav2_navfn_planner::NavfnPlanner',
                 'tolerance': 0.1, 'use_astar': False, 'allow_unknown': False},
    # NavFn with A* heuristic
    'astar': {'plugin': 'nav2_navfn_planner::NavfnPlanner',
              'tolerance': 0.1, 'use_astar': True, 'allow_unknown': False},
    # Smac 2D: cost-aware A* on the 2-D grid, 8-connected
    'smac2d': {'plugin': 'nav2_smac_planner::SmacPlanner2D',
               'tolerance': 0.1, 'downsample_costmap': False, 'downsampling_factor': 1,
               'allow_unknown': False, 'max_iterations': 1000000,
               'max_on_approach_iterations': 1000, 'max_planning_time': 2.0,
               'cost_travel_multiplier': 2.0, 'use_final_approach_orientation': False,
               'smoother': {'max_iterations': 1000, 'w_smooth': 0.3, 'w_data': 0.2,
                            'tolerance': 1.0e-10}},
}


def _rewrite_params(context):
    """Load the Nav2 YAML, apply launch-arg overrides, write a temp copy."""
    src = LaunchConfiguration('params_file').perform(context)
    planner = LaunchConfiguration('planner').perform(context)
    max_vel = float(LaunchConfiguration('max_vel').perform(context))
    inflation = float(LaunchConfiguration('inflation_radius').perform(context))
    x = float(LaunchConfiguration('x_pose').perform(context))
    y = float(LaunchConfiguration('y_pose').perform(context))
    yaw = float(LaunchConfiguration('yaw').perform(context))

    if planner not in PLANNERS:
        raise RuntimeError(f'planner must be one of {list(PLANNERS)}, got "{planner}"')

    with open(src) as f:
        p = yaml.safe_load(f)

    p['planner_server']['ros__parameters']['GridBased'] = PLANNERS[planner]

    fp = p['controller_server']['ros__parameters']['FollowPath']
    fp['max_vel_x'] = max_vel
    fp['max_speed_xy'] = max_vel

    for cm in ('local_costmap', 'global_costmap'):
        p[cm][cm]['ros__parameters']['inflation_layer']['inflation_radius'] = inflation

    p['amcl']['ros__parameters']['initial_pose'] = {'x': x, 'y': y, 'z': 0.0, 'yaw': yaw}

    out = os.path.join(tempfile.gettempdir(), f'tb3_maze_nav2_{planner}.yaml')
    with open(out, 'w') as f:
        yaml.safe_dump(p, f, sort_keys=False)

    map_yaml = LaunchConfiguration('map').perform(context)
    actions = [LogInfo(msg=f'[tb3_maze] planner={planner} max_vel={max_vel} '
                           f'inflation={inflation} params={out}')]
    if not os.path.exists(map_yaml):
        fallback = os.path.join(get_package_share_directory('tb3_maze'),
                                'maps', 'maze_groundtruth.yaml')
        actions.append(LogInfo(msg=f'[tb3_maze] WARNING: {map_yaml} not found; using the '
                                   f'ground-truth map {fallback}. Save the SLAM map for the '
                                   f'real runs.'))
        map_yaml = fallback

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')),
        launch_arguments={'map': map_yaml, 'use_sim_time': 'true',
                          'params_file': out, 'autostart': 'true'}.items())
    return actions + [nav2]


def generate_launch_description():
    pkg = get_package_share_directory('tb3_maze')
    rviz_cfg = os.path.join(get_package_share_directory('turtlebot3_navigation2'),
                            'rviz', 'tb3_navigation2.rviz')

    return LaunchDescription([
        DeclareLaunchArgument('sim', default_value='true', description='also start Gazebo'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('metrics', default_value='true',
                              description='run metrics_logger (results/runs.csv)'),
        DeclareLaunchArgument('map', default_value=os.path.join(pkg, 'maps', 'maze_map.yaml')),
        DeclareLaunchArgument('params_file',
                              default_value=os.path.join(pkg, 'config', 'nav2_params_maze.yaml')),
        DeclareLaunchArgument('planner', default_value='dijkstra',
                              description='dijkstra | astar | smac2d'),
        DeclareLaunchArgument('max_vel', default_value='0.22'),
        DeclareLaunchArgument('inflation_radius', default_value='0.22'),
        DeclareLaunchArgument('x_pose', default_value='0.0'),
        DeclareLaunchArgument('y_pose', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('method', default_value='',
                              description='label in runs.csv (default nav2_<planner>)'),
        DeclareLaunchArgument('results_dir', default_value=os.path.expanduser('~/tb3_maze_results')),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'maze.launch.py')),
            launch_arguments={'headless': LaunchConfiguration('headless'),
                              'x_pose': LaunchConfiguration('x_pose'),
                              'y_pose': LaunchConfiguration('y_pose'),
                              'yaw': LaunchConfiguration('yaw')}.items(),
            condition=IfCondition(LaunchConfiguration('sim'))),

        OpaqueFunction(function=_rewrite_params),

        Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
             arguments=['-d', rviz_cfg], parameters=[{'use_sim_time': True}],
             condition=IfCondition(LaunchConfiguration('rviz'))),

        OpaqueFunction(function=_metrics_node),
    ])


def _metrics_node(context):
    if LaunchConfiguration('metrics').perform(context).lower() != 'true':
        return []
    method = (LaunchConfiguration('method').perform(context)
              or 'nav2_' + LaunchConfiguration('planner').perform(context))
    return [Node(package='tb3_maze', executable='metrics_logger', output='screen',
                 parameters=[{'use_sim_time': True, 'method': method,
                              'results_dir': LaunchConfiguration('results_dir')
                              .perform(context)}])]
