"""
slam_sim.launch.py — Gazebo + SLAM mapping in one command.

Usage:
    ros2 launch bringup slam_sim.launch.py
    ros2 launch bringup slam_sim.launch.py world_name:=warehouse
    ros2 launch bringup slam_sim.launch.py map_path:=~/ros2_maps/my_map

Drive with the xterm teleop window.  Press Ctrl+C when done — the map is
saved automatically to ~/ros2_maps/warehouse.pgm / .yaml.

To save mid-session without stopping:
    ros2 service call /map_saver/save_map std_srvs/srv/Trigger {}
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


def generate_launch_description():

    pkg_description = get_package_share_directory('description')
    mcr2_urdf = os.path.join(pkg_description, 'urdf', 'puzzlebot_mcr2.urdf.xacro')

    world_name_arg = DeclareLaunchArgument(
        'world_name',
        default_value='warehouse',
        description='Gazebo world to load (no extension)',
    )

    map_path_arg = DeclareLaunchArgument(
        'map_path',
        default_value=os.path.expanduser('~/ros2_maps/warehouse'),
        description='Output map base path (no extension)',
    )

    # ── 1. Gazebo + robot (RSP, bridge, controller spawners) ─────────
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_description, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world_name': LaunchConfiguration('world_name'),
            'model': mcr2_urdf,
        }.items(),
    )

    # ── 2. Mapping stack (delayed 22 s to let controllers come up) ────
    # gazebo.launch.py spawns joint_state_broadcaster at 12 s and
    # puzzlebot_controller at 18 s — mapping needs /puzzlebot_controller/odom.
    mapping_launch = TimerAction(
        period=22.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('slam'),
                        'launch', 'mapping.launch.py',
                    )
                ),
                launch_arguments={
                    'map_path': LaunchConfiguration('map_path'),
                }.items(),
            )
        ],
    )

    return LaunchDescription([
        world_name_arg,
        map_path_arg,
        gazebo_launch,
        mapping_launch,
    ])
