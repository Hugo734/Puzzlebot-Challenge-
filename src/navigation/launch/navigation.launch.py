"""
navigation.launch.py — Launch SLAM + nav_node + RViz.

Assumes Gazebo is already running separately.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_nav  = get_package_share_directory('navigation')
    pkg_slam = get_package_share_directory('slam')

    nav_params  = os.path.join(pkg_nav,  'config', 'nav_params.yaml')
    slam_params = os.path.join(pkg_slam, 'config', 'slam_params.yaml')
    rviz_cfg    = os.path.join(pkg_slam, 'config', 'slam.rviz')

    # ── SLAM node ──────────────────────────────────────────────────────
    slam_node = Node(
        package='slam',
        executable='slam_node',
        name='slam_node',
        parameters=[slam_params, {
            'use_sim_time': True,
            'use_icp': False,
        }],
        remappings=[('/odom', '/puzzlebot_controller/odom')],
        output='screen',
        emulate_tty=True,
    )

    # ── Navigation node ────────────────────────────────────────────────
    nav_node = Node(
        package='navigation',
        executable='nav_node',
        name='nav_node',
        parameters=[nav_params, {'use_sim_time': True}],
        output='screen',
        emulate_tty=True,
    )

    # ── RViz2 ──────────────────────────────────────────────────────────
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_cfg],
        parameters=[{'use_sim_time': True}],
        additional_env={'MESA_GL_VERSION_OVERRIDE': '3.3COMPAT'},
        output='screen',
    )

    return LaunchDescription([
        slam_node,
        nav_node,
        rviz_node,
    ])
