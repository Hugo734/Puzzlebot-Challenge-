"""Launch file for the perception package."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory('perception')
    default_params = os.path.join(pkg_share, 'config', 'vision_params.yaml')

    # ── Launch arguments ──────────────────────────────────────────────
    use_mock_arg = DeclareLaunchArgument(
        'use_mock_camera',
        default_value='false',
        description='Use synthetic frames instead of a real camera (for simulation / CI)',
    )
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Full path to the vision parameters YAML file',
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use /clock topic for ROS time (Gazebo simulation)',
    )

    # ── Nodes ─────────────────────────────────────────────────────────
    alignment_node = Node(
        package='perception',
        executable='alignment_node',
        name='alignment_node',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'use_mock_camera': LaunchConfiguration('use_mock_camera'),
                'use_sim_time':    LaunchConfiguration('use_sim_time'),
            },
        ],
    )

    return LaunchDescription([
        use_mock_arg,
        params_file_arg,
        use_sim_time_arg,
        alignment_node,
    ])
