"""
real.launch.py — Real hardware stack for the Puzzlebot AMR (Jetson Nano).

Launches (all with use_sim_time:=false, no Gazebo, no RViz):
  1. Differential-drive controller (controller/controller.launch.py)
  2. SLAM node (slam package, odom remapped)
  3. EKF localization (localization/ekf_localization.launch.py, mode:=real)
  4. Navigation node (navigation package)
  5. Perception / alignment node (perception package)
  6. Lifter node with real Jetson GPIO (lifting package)
  7. State machine / mission control (mission_control package)
  8. Dashboard web server (dashboard package)
  9. Voice control node (voice_control package)

Usage:
  ros2 launch bringup real.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    # ── 1. Differential-drive controller ─────────────────────────────
    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('controller'), 'launch'
            ),
            '/controller.launch.py',
        ]),
        launch_arguments={
            'use_sim_time': 'false',
        }.items(),
    )

    # ── 2. SLAM node ─────────────────────────────────────────────────
    slam_node = Node(
        package='slam',
        executable='slam_node',
        name='slam_node',
        parameters=[{'use_sim_time': False}],
        remappings=[('/odom', '/puzzlebot_controller/odom')],
        output='screen',
    )

    # ── 3. EKF localization ───────────────────────────────────────────
    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('localization'), 'launch'
            ),
            '/ekf_localization.launch.py',
        ]),
        launch_arguments={
            'mode': 'real',
        }.items(),
    )

    # ── 4. Navigation node ────────────────────────────────────────────
    navigation_node = Node(
        package='navigation',
        executable='navigation_node',
        name='navigation_node',
        parameters=[{'use_sim_time': False, 'use_mock': False}],
        output='screen',
    )

    # ── 5. Perception / alignment node ───────────────────────────────
    alignment_node = Node(
        package='perception',
        executable='alignment_node',
        name='alignment_node',
        parameters=[{'use_sim_time': False}],
        output='screen',
    )

    # ── 6. Lifter node (real Jetson.GPIO, 3-bit FPGA encoding) ───────
    lifting_node = Node(
        package='lifting',
        executable='lifting_node',
        name='lifting_node',
        parameters=[{'use_sim_time': False, 'use_mock_gpio': False}],
        output='screen',
    )

    # ── 7. State machine / mission control ────────────────────────────
    state_machine_node = Node(
        package='mission_control',
        executable='state_machine_node',
        name='state_machine_node',
        parameters=[{'use_sim_time': False}],
        output='screen',
    )

    # ── 8. Dashboard / web server ─────────────────────────────────────
    dashboard_node = Node(
        package='dashboard',
        executable='dashboard_node',
        name='dashboard_node',
        parameters=[{'use_sim_time': False}],
        output='screen',
    )

    # ── 9. Voice control (LPC + VQ pipeline) ─────────────────────────
    voice_node = Node(
        package='voice_control',
        executable='voice_node',
        name='voice_node',
        parameters=[{'use_sim_time': False}],
        output='screen',
    )

    return LaunchDescription([
        controller_launch,
        slam_node,
        ekf_launch,
        navigation_node,
        alignment_node,
        lifting_node,
        state_machine_node,
        dashboard_node,
        voice_node,
    ])
