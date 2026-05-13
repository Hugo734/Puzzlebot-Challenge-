import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_slam = get_package_share_directory('slam')
    rviz_cfg = os.path.join(pkg_slam, 'config', 'slam.rviz')

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

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_cfg],
        output='screen',
        additional_env={'MESA_GL_VERSION_OVERRIDE': '3.3COMPAT'},
    )

    return LaunchDescription([
        map_yaml_arg,
        output_arg,
        rviz,
    ])
