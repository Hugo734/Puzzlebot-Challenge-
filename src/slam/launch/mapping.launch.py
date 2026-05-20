"""
Mapping launch (Gazebo / simulation).

Start Gazebo first:
    ros2 launch description gazebo.launch.py world_name:=new_warehouse

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
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_slam        = get_package_share_directory('slam')
    pkg_description = get_package_share_directory('description')

    slam_params = os.path.join(pkg_slam, 'config', 'slam_params.yaml')
    rviz_cfg    = os.path.join(pkg_slam, 'config', 'slam.rviz')
    urdf_path   = os.path.join(pkg_description, 'urdf', 'puzzlebot_with_lifter.urdf.xacro')

    map_path_arg = DeclareLaunchArgument(
        'map_path',
        default_value=os.path.expanduser('~/ros2_maps/warehouse'),
        description='Output map base path (no extension)',
    )

    # odom → base_footprint: normally published by diff_drive_controller once
    # Gazebo activates.  This static publisher serves as an immediate fallback
    # so the full TF chain (map→odom→base_footprint→base_link→lidar_link) is
    # valid from launch-time. The controller's dynamic TF takes precedence once
    # the robot starts moving.
    odom_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_base_footprint',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint'],
        output='screen',
    )

    # TF tree: map → odom → base_footprint → base_link → lidar_link / wheel_*
    # Gazebo's diff_drive controller publishes odom → base_footprint dynamically.
    # This RSP node covers the static joints above base_footprint.
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(
                Command(['xacro ', urdf_path, ' is_sim:=true is_ignition:=true']),
                value_type=str,
            ),
            'use_sim_time': True,
        }],
        output='screen',
    )

    slam_node = Node(
        package='slam',
        executable='slam_node',
        name='slam_node',
        parameters=[
            slam_params,
            {
                'use_sim_time': True,
                # Pure-odometry mode: Gazebo odometry is clean; ICP adds
                # latency without improving initial map quality in sim.
                'use_icp': False,
                # Publish map→odom TF from the very first scan so RViz can
                # display the robot model before any movement.
                'min_delta_xy':    0.0,
                'min_delta_theta': 0.0,
            },
        ],
        # gz_ros2_control's diff_drive publishes odom on this topic.
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
        odom_to_base,
        robot_state_publisher,
        slam_node,
        map_saver,
        rviz,
        teleop,
    ])
