import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_nav  = get_package_share_directory('navigation')
    rviz_cfg = os.path.join(pkg_nav, 'config', 'waypoint_recorder.rviz')

    map_yaml_arg = DeclareLaunchArgument(
        'map_yaml',
        default_value=os.path.expanduser('~/ros2_maps/warehouse.yaml'),
        description='Path to the saved map YAML',
    )
    output_arg = DeclareLaunchArgument(
        'output',
        default_value=os.path.expanduser('~/ros2_maps/waypoints.yaml'),
        description='Where to write waypoints.yaml',
    )

    # Static TF: map frame exists without needing a full robot stack running.
    # Publishes identity map→odom so RViz fixed frame resolves cleanly.
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_odom_static_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen',
    )

    waypoint_recorder = Node(
        package='navigation',
        executable='waypoint_recorder',
        name='waypoint_recorder',
        parameters=[{
            'map_yaml': LaunchConfiguration('map_yaml'),
            'output':   LaunchConfiguration('output'),
        }],
        output='screen',
        prefix='xterm -e',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_cfg],
        output='screen',
    )

    return LaunchDescription([
        map_yaml_arg,
        output_arg,
        static_tf,
        waypoint_recorder,
        rviz,
    ])
