"""
real.launch.py — Single-command real-robot stack for the Puzzlebot AMR
(Jetson Nano + MCR2 Puzzlebot).

Brings up everything needed on hardware:

  * Real-robot controller (real_odom + vel_smoother)
      vel_smoother ramps /cmd_vel_in → /cmd_vel so the powerbank does not
      brown out the hackerboard on velocity steps.  All upstream nodes
      must publish to /cmd_vel_in (already wired in nav/perception/dashboard).
  * EKF localization (mode=real → velocity_bridge + ekf + icp + RSP)
  * SLAM (slam_node) with runtime MAPPING ↔ NAVIGATION mode switching
  * map_saver_node — `/map_saver/save_map` Trigger service
  * Navigation (nav_node)
  * Mission control state machine
  * Dashboard (Flask + SocketIO)
  * Lifting node (real Jetson.GPIO, 3-bit FPGA encoding)
  * Voice control (LPC + VQ)

Usage:
  ros2 launch bringup real.launch.py
  ros2 launch bringup real.launch.py start_mode:=mapping
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_controller    = get_package_share_directory('controller')
    pkg_localization  = get_package_share_directory('localization')
    pkg_slam          = get_package_share_directory('slam')
    pkg_nav           = get_package_share_directory('navigation')

    map_yaml_default       = os.path.expanduser('~/ros2_maps/warehouse.yaml')
    waypoints_yaml_default = os.path.expanduser('~/ros2_maps/waypoints.yaml')
    slam_params_path       = os.path.join(pkg_slam, 'config', 'slam_params.yaml')
    nav_params_path        = os.path.join(pkg_nav,  'config', 'nav_params.yaml')
    rviz_cfg_path          = os.path.join(pkg_slam, 'config', 'slam.rviz')

    # ── Args ──────────────────────────────────────────────────────────
    start_mode_arg = DeclareLaunchArgument(
        'start_mode',
        default_value='mapping',
        description='Initial system mode (mapping | navigation).',
    )
    map_yaml_arg = DeclareLaunchArgument(
        'map_yaml', default_value='',
        description='Path to saved map yaml (empty string = start fresh, no preload)',
    )
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Launch RViz2 with the SLAM config',
    )

    start_mode = LaunchConfiguration('start_mode')
    map_yaml   = LaunchConfiguration('map_yaml')

    # ── 1. Controller (real_odom + twist_relay) ───────────────────────
    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_controller, 'launch', 'real.launch.py')
        ),
    )

    # ── 2. EKF localization (mode=real) — RSP, velocity_bridge, EKF ───
    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_localization, 'launch', 'ekf_localization.launch.py')
        ),
        launch_arguments={'mode': 'real', 'open_rviz': 'false', 'use_icp': 'false'}.items(),
    )

    # ── 3. SLAM ───────────────────────────────────────────────────────
    # Delay 5 s so EKF + RSP are publishing TF before SLAM starts consuming /scan.
    slam_node = TimerAction(period=5.0, actions=[Node(
        package='slam',
        executable='slam_node',
        name='slam_node',
        parameters=[slam_params_path, {
            'use_sim_time': False,
            'map_yaml':     map_yaml,
            'start_mode':   start_mode,
        }],
        output='screen',
        emulate_tty=True,
    )])

    # ── 4. Map saver ──────────────────────────────────────────────────
    map_saver_node = TimerAction(period=6.0, actions=[Node(
        package='slam',
        executable='map_saver',
        name='map_saver',
        parameters=[{
            'use_sim_time': False,
            'map_path':     os.path.splitext(map_yaml_default)[0],
        }],
        output='screen',
    )])

    # ── 5. Navigation ─────────────────────────────────────────────────
    nav_node = TimerAction(period=6.0, actions=[Node(
        package='navigation',
        executable='nav_node',
        name='nav_node',
        parameters=[nav_params_path, {
            'use_sim_time':   False,
            'map_yaml':       map_yaml,
            'waypoints_yaml': waypoints_yaml_default,
            'start_mode':     start_mode,
        }],
        output='screen',
        emulate_tty=True,
    )])

    # ── 6. Mission control ────────────────────────────────────────────
    mission_node = TimerAction(period=6.0, actions=[Node(
        package='mission_control',
        executable='state_machine_node',
        name='state_machine_node',
        parameters=[{'use_sim_time': False}],
        output='screen',
    )])

    # ── 7. Dashboard ──────────────────────────────────────────────────
    dashboard_node = TimerAction(period=6.0, actions=[Node(
        package='dashboard',
        executable='dashboard_node',
        name='dashboard_node',
        parameters=[{
            'use_sim_time': False,
            'start_mode':   start_mode,
        }],
        output='screen',
    )])

    # ── 8. Lifter (real Jetson.GPIO, 3-bit FPGA encoding) ─────────────
    lifting_node = Node(
        package='lifting',
        executable='lifting_node',
        name='lifting_node',
        parameters=[{'use_sim_time': False, 'use_mock_gpio': False}],
        output='screen',
    )

    # ── 9. Voice control ──────────────────────────────────────────────
    voice_node = Node(
        package='voice_control',
        executable='voice_node',
        name='voice_node',
        parameters=[{'use_sim_time': False}],
        output='screen',
    )

    # ── 10. RViz2 (SLAM config) ───────────────────────────────────────
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_cfg_path],
        parameters=[{'use_sim_time': False}],
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        start_mode_arg,
        map_yaml_arg,
        rviz_arg,
        controller_launch,
        ekf_launch,
        slam_node,
        map_saver_node,
        nav_node,
        mission_node,
        # dashboard_node,
        lifting_node,
        voice_node,
        rviz_node,
    ])
