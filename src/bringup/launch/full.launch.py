"""
full.launch.py — Unified entry point for the Puzzlebot AMR system.

Accepts:
  sim:=true  (default) — launches sim.launch.py (Gazebo + full sim stack)
  sim:=false           — launches real.launch.py (Jetson Nano hardware stack)

Usage:
  ros2 launch bringup full.launch.py               # simulation (default)
  ros2 launch bringup full.launch.py sim:=true     # simulation (explicit)
  ros2 launch bringup full.launch.py sim:=false    # real hardware
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    sim_arg = DeclareLaunchArgument(
        'sim',
        default_value='true',
        description='Set to "true" to launch the Gazebo simulation stack, '
                    '"false" to launch the real Jetson Nano hardware stack.',
    )

    sim = LaunchConfiguration('sim')

    pkg_bringup = get_package_share_directory('bringup')

    # ── Simulation stack (sim:=true) ──────────────────────────────────
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, 'launch', 'sim.launch.py')
        ),
        condition=IfCondition(sim),
    )

    # ── Real hardware stack (sim:=false) ──────────────────────────────
    real_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, 'launch', 'real.launch.py')
        ),
        condition=UnlessCondition(sim),
    )

    return LaunchDescription([
        sim_arg,
        sim_launch,
        real_launch,
    ])
