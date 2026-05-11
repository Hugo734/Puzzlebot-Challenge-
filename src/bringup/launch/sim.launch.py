"""
sim.launch.py — Full simulation stack for the Puzzlebot AMR warehouse demo.

Launches (all with use_sim_time:=true):
  1. Gazebo with warehouse world (description/gazebo2.launch.py)
  2. Differential-drive controller (controller/controller.launch.py)
  3. SLAM node (slam package, odom remapped)
  4. EKF localization (localization/ekf_localization.launch.py, mode:=sim)
  5. Navigation node (navigation package)
  6. Perception / alignment node (perception package)
  7. Lifter node with mock GPIO (lifting package)
  8. State machine / mission control (mission_control package)
  9. Dashboard web server (dashboard package)
  10. RViz2

Usage:
  ros2 launch bringup sim.launch.py
  ros2 launch bringup sim.launch.py world_name:=warehouse
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    world_name_arg = DeclareLaunchArgument(
        'world_name',
        default_value='warehouse',
        description='Gazebo world name to load (e.g. warehouse, obstacles, empty)',
    )

    world_name = LaunchConfiguration('world_name')

    # ── 1. Gazebo + robot URDF ────────────────────────────────────────
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('description'), 'launch'
            ),
            '/gazebo2.launch.py',
        ]),
        launch_arguments={
            'world_name': world_name,
        }.items(),
    )

    # ── 2. Differential-drive controller ─────────────────────────────
    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('controller'), 'launch'
            ),
            '/controller.launch.py',
        ]),
        launch_arguments={
            'use_sim_time': 'true',
        }.items(),
    )

    # ── 3. SLAM node ─────────────────────────────────────────────────
    slam_node = Node(
        package='slam',
        executable='slam_node',
        name='slam_node',
        parameters=[{'use_sim_time': True}],
        remappings=[('/odom', '/puzzlebot_controller/odom')],
        output='screen',
    )

    # ── 4. EKF localization ───────────────────────────────────────────
    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('localization'), 'launch'
            ),
            '/ekf_localization.launch.py',
        ]),
        launch_arguments={
            'mode': 'sim',
        }.items(),
    )

    # ── 5. Navigation node ────────────────────────────────────────────
    navigation_node = Node(
        package='navigation',
        executable='navigation_node',
        name='navigation_node',
        parameters=[{'use_sim_time': True, 'use_mock': False}],
        output='screen',
    )

    # ── 6. Perception / alignment node ───────────────────────────────
    alignment_node = Node(
        package='perception',
        executable='alignment_node',
        name='alignment_node',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # ── 7. Lifter node (mock GPIO for simulation) ─────────────────────
    lifting_node = Node(
        package='lifting',
        executable='lifting_node',
        name='lifting_node',
        parameters=[{'use_sim_time': True, 'use_mock_gpio': True}],
        output='screen',
    )

    # ── 8. State machine / mission control ────────────────────────────
    state_machine_node = Node(
        package='mission_control',
        executable='state_machine_node',
        name='state_machine_node',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # ── 9. Dashboard / web server ─────────────────────────────────────
    dashboard_node = Node(
        package='dashboard',
        executable='dashboard_node',
        name='dashboard_node',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # ── 10. RViz2 ─────────────────────────────────────────────────────
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    return LaunchDescription([
        world_name_arg,
        gazebo_launch,
        controller_launch,
        slam_node,
        ekf_launch,
        navigation_node,
        alignment_node,
        lifting_node,
        state_machine_node,
        dashboard_node,
        rviz_node,
    ])
