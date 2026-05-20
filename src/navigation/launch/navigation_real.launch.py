"""
Navigation launch — real Puzzlebot hardware (Jetson Nano).

Prerequisites:
  1. Map already saved:     ~/ros2_maps/warehouse.{pgm,yaml}
  2. Waypoints already saved: ~/ros2_maps/waypoints.yaml
  3. LiDAR driver running:  ros2 launch rplidar_ros rplidar_a1_launch.py serial_port:=/dev/ttyUSB0

Then start navigation:
    ros2 launch navigation navigation_real.launch.py

Send a goal (separate terminal):
    ros2 topic pub --once /goal_waypoint std_msgs/String '{data: "truck_1"}'

Monitor status:
    ros2 topic echo /nav_status

Cancel current goal:
    ros2 topic pub --once /goal_waypoint std_msgs/String '{data: "stop"}'
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_slam        = get_package_share_directory('slam')
    pkg_nav         = get_package_share_directory('navigation')
    pkg_description = get_package_share_directory('description')

    slam_params = os.path.join(pkg_slam, 'config', 'slam_params.yaml')
    nav_params  = os.path.join(pkg_nav,  'config', 'nav_params.yaml')
    rviz_cfg    = os.path.join(pkg_slam, 'config', 'slam.rviz')
    urdf_path   = os.path.join(pkg_description, 'urdf', 'puzzlebot_with_lifter.urdf.xacro')

    wheel_radius_arg = DeclareLaunchArgument(
        'wheel_radius', default_value='0.05',
        description='Wheel radius in metres')
    wheel_separation_arg = DeclareLaunchArgument(
        'wheel_separation', default_value='0.19',
        description='Wheel centre-to-centre distance in metres')
    map_yaml_arg = DeclareLaunchArgument(
        'map_yaml',
        default_value=os.path.expanduser('~/ros2_maps/warehouse.yaml'),
        description='Saved map YAML path')
    waypoints_yaml_arg = DeclareLaunchArgument(
        'waypoints_yaml',
        default_value=os.path.expanduser('~/ros2_maps/waypoints.yaml'),
        description='Saved waypoints YAML path')

    clear_fastrtps_profile = SetEnvironmentVariable(
        name='FASTRTPS_DEFAULT_PROFILES_FILE', value='')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(
                Command(['xacro ', urdf_path, ' is_sim:=false is_ignition:=false']),
                value_type=str,
            ),
            'use_sim_time': False,
        }],
        output='screen',
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': False}],
        output='screen',
    )

    velocity_bridge = Node(
        package='localization',
        executable='velocity_bridge',
        name='velocity_bridge',
        output='screen',
    )

    real_odom = Node(
        package='controller',
        executable='real_odom',
        name='real_odom',
        parameters=[{
            'wheel_radius':     LaunchConfiguration('wheel_radius'),
            'wheel_separation': LaunchConfiguration('wheel_separation'),
            'use_sim_time':     False,
        }],
        output='screen',
    )

    lidar_frame_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_frame_laser',
        arguments=['--frame-id', 'lidar_link', '--child-frame-id', 'laser'],
        output='screen',
    )

    lidar_frame_laser_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_frame_laser_link',
        arguments=['--frame-id', 'lidar_link', '--child-frame-id', 'laser_link'],
        output='screen',
    )

    scan_fix = Node(
        package='slam',
        executable='scan_timestamp_fix',
        name='scan_timestamp_fix',
        output='screen',
    )

    # SLAM runs in localization mode: the saved map primes the log-odds grid
    # so ICP has structure to match against from the first scan. The robot
    # pose estimate flows out on /slam_pose → nav_node.
    # Use RViz "2D Pose Estimate" button to correct the initial pose if needed.
    slam_node = Node(
        package='slam',
        executable='slam_node',
        name='slam_node',
        parameters=[
            slam_params,
            {
                'use_sim_time': False,
                'map_yaml':     LaunchConfiguration('map_yaml'),
            },
        ],
        remappings=[
            ('/odom', '/puzzlebot_controller/odom'),
            ('/scan', '/scan_fixed'),
        ],
        output='screen',
    )

    nav_node = Node(
        package='navigation',
        executable='nav_node',
        name='nav_node',
        parameters=[
            nav_params,
            {
                'use_sim_time':    False,
                'map_yaml':        LaunchConfiguration('map_yaml'),
                'waypoints_yaml':  LaunchConfiguration('waypoints_yaml'),
            },
        ],
        remappings=[('/scan', '/scan_fixed')],
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_cfg],
        parameters=[{'use_sim_time': False}],
        output='screen',
    )

    return LaunchDescription([
        clear_fastrtps_profile,
        wheel_radius_arg,
        wheel_separation_arg,
        map_yaml_arg,
        waypoints_yaml_arg,
        robot_state_publisher,
        joint_state_publisher,
        velocity_bridge,
        real_odom,
        lidar_frame_laser,
        lidar_frame_laser_link,
        scan_fix,
        slam_node,
        nav_node,
        rviz,
    ])
