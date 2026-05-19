"""
nav_sim.launch.py — Gazebo simulation + navigation with a pre-saved map.

Usage:
    ros2 launch bringup nav_sim.launch.py
    ros2 launch bringup nav_sim.launch.py world_name:=warehouse
    ros2 launch bringup nav_sim.launch.py map_yaml:=~/ros2_maps/warehouse.yaml

The nav_node reads the map from map_yaml, plans paths with A*, and follows
them with the Bug1 + PID controller.  SLAM runs in parallel to keep the
robot localized inside the known map.

Send a navigation goal via the /mission topic:
    ros2 topic pub /mission std_msgs/msg/String \
        '{"data": "{\"source\": \"rack_1\", \"dest\": \"truck_1\", \"pallet_id\": 0}"}'
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_nav  = get_package_share_directory('navigation')
    pkg_slam = get_package_share_directory('slam')
    pkg_bringup = get_package_share_directory('bringup')

    nav_params  = os.path.join(pkg_nav,  'config', 'nav_params.yaml')
    slam_params = os.path.join(pkg_slam, 'config', 'slam_params.yaml')
    rviz_cfg    = os.path.join(pkg_nav,  'config', 'nav.rviz')

    world_name_arg = DeclareLaunchArgument(
        'world_name',
        default_value='warehouse',
        description='Gazebo world to load (no extension)',
    )

    map_yaml_arg = DeclareLaunchArgument(
        'map_yaml',
        default_value=os.path.expanduser('~/ros2_maps/warehouse.yaml'),
        description='Absolute path to the saved map YAML file',
    )

    # ── 1. Gazebo + robot + controller ───────────────────────────────────
    simulated_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, 'launch', 'simulated_robot.launch.py')
        ),
        launch_arguments={
            'world_name': LaunchConfiguration('world_name'),
            'use_simple_controller': 'true',
        }.items(),
    )

    # ── 2. Navigation stack (delayed to let the controller come up) ───────
    slam_node = Node(
        package='slam',
        executable='slam_node',
        name='slam_node',
        parameters=[
            slam_params,
            {
                'use_sim_time': True,
                'use_icp': False,
                'min_delta_xy':    0.0,
                'min_delta_theta': 0.0,
            },
        ],
        remappings=[('/odom', '/puzzlebot_controller/odom')],
        output='screen',
        emulate_tty=True,
    )

    nav_node = Node(
        package='navigation',
        executable='nav_node',
        name='nav_node',
        parameters=[
            nav_params,
            {
                'use_sim_time': True,
                'map_yaml': LaunchConfiguration('map_yaml'),
            },
        ],
        output='screen',
        emulate_tty=True,
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_cfg],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    nav_stack = TimerAction(
        period=10.0,
        actions=[slam_node, nav_node, rviz],
    )

    return LaunchDescription([
        world_name_arg,
        map_yaml_arg,
        simulated_robot,
        nav_stack,
    ])
