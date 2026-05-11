import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_nav = get_package_share_directory('navigation')
    nav_params = os.path.join(pkg_nav, 'config', 'nav_params.yaml')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock when true',
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    navigation_node = Node(
        package='navigation',
        executable='navigation_node',
        name='navigation_node',
        parameters=[nav_params, {'use_sim_time': use_sim_time}],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        use_sim_time_arg,
        navigation_node,
    ])
