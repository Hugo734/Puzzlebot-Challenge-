"""Launch file for the lifting (FPGA lifter GPIO control) node."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    use_mock_gpio = LaunchConfiguration('use_mock_gpio').perform(context)

    pkg_lifting = get_package_share_directory('lifting')
    params_file = os.path.join(pkg_lifting, 'config', 'lifting_params.yaml')

    lifting_node = Node(
        package='lifting',
        executable='lifting_node',
        name='lifting_node',
        parameters=[
            params_file,
            {'use_mock_gpio': use_mock_gpio.lower() in ('true', '1', 'yes')},
        ],
        output='screen',
    )

    return [lifting_node]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_mock_gpio',
            default_value='true',
            description=(
                'Set to "false" to use real Jetson.GPIO hardware. '
                'Keep "true" for development/simulation.'
            ),
        ),
        OpaqueFunction(function=launch_setup),
    ])
