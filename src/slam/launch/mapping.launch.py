"""
Mapping launch — assumes Gazebo is already running.

Start Gazebo first:
    ros2 launch description gazebo.launch.py world_name:=warehouse

Then start mapping:
    ros2 launch slam mapping.launch.py

Drive with the xterm teleop window.  When done, press Ctrl+C — the map is
saved automatically to ~/ros2_maps/warehouse.pgm / .yaml.

To save mid-session without stopping:
    ros2 service call /map_saver/save_map std_srvs/srv/Trigger {}
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_slam    = get_package_share_directory('slam')
    slam_params = os.path.join(pkg_slam, 'config', 'slam_params.yaml')
    rviz_cfg    = os.path.join(pkg_slam, 'config', 'slam.rviz')

    map_path_arg = DeclareLaunchArgument(
        'map_path',
        default_value=os.path.expanduser('~/ros2_maps/warehouse'),
        description='Output map base path (no extension)',
    )

    slam_node = Node(
        package='slam',
        executable='slam_node',
        name='slam_node',
        parameters=[
            slam_params,
            {
                'use_sim_time': True,
                # Pure-odometry mode for mapping in sim: odometry is clean and
                # ICP adds latency without improving the initial map quality.
                'use_icp': False,
            },
        ],
        remappings=[('/odom', '/puzzlebot_controller/odom')],
        output='screen',
    )

    map_saver = Node(
        package='slam',
        executable='map_saver',
        name='map_saver',
        parameters=[{
            'use_sim_time': True,
            'map_path': LaunchConfiguration('map_path'),
        }],
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_cfg],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    teleop = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop',
        output='screen',
        prefix='xterm -e',
        remappings=[('cmd_vel', '/cmd_vel')],
    )

    return LaunchDescription([
        map_path_arg,
        slam_node,
        map_saver,
        rviz,
        teleop,
    ])
