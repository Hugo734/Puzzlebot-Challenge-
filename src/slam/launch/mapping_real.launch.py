"""
Mapping launch — real Puzzlebot hardware (Jetson Nano).

Start the LiDAR driver first (must publish /scan):
    ros2 launch rplidar_ros rplidar_a1_launch.py serial_port:=/dev/ttyUSB0

Then start mapping (two-terminal mode):
    ros2 launch slam mapping_real.launch.py

Drive with the xterm teleop window.  When done, press Ctrl+C — the map is
saved automatically to ~/ros2_maps/warehouse.pgm / .yaml.

To save mid-session without stopping:
    ros2 service call /map_saver/save_map std_srvs/srv/Trigger {}

If the map appears tilted, tune map_initial_heading / laser_yaw_trim in
config/slam_params.yaml.
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

    wheel_radius_arg = DeclareLaunchArgument(
        'wheel_radius', default_value='0.05',
        description='Wheel radius in metres')
    wheel_separation_arg = DeclareLaunchArgument(
        'wheel_separation', default_value='0.19',
        description='Wheel centre-to-centre distance in metres')

    # TF tree: odom → base_footprint → base_link → lidar_link / wheel_*
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

    # Publishes zero wheel joint states so robot_state_publisher can resolve
    # wheel_left_link / wheel_right_link transforms for RViz visualization.
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': False}],
        output='screen',
    )

    # Bridges /VelocityEncl + /VelocityEncnR encoder topics → /wl /wr (Float32, rad/s).
    velocity_bridge = Node(
        package='localization',
        executable='velocity_bridge',
        name='velocity_bridge',
        output='screen',
    )

    # Integrates /wl + /wr → /puzzlebot_controller/odom (Odometry) and
    # publishes the dynamic odom → base_footprint TF.
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

    # Always bridge both common RPLidar A1 frame names to lidar_link so SLAM
    # and RViz work regardless of which frame_id the driver publishes.
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

    # Re-stamps /scan with local clock so TF lookups work despite robot/PC
    # clock offset (~200-400 ms from SSH latency).
    scan_fix = Node(
        package='slam',
        executable='scan_timestamp_fix',
        name='scan_timestamp_fix',
        output='screen',
    )

    slam_node = Node(
        package='slam',
        executable='slam_node',
        name='slam_node',
        parameters=[
            slam_params,
            {'use_sim_time': False},
        ],
        remappings=[
            ('/odom', '/puzzlebot_controller/odom'),
            ('/scan', '/scan_fixed'),
        ],
        output='screen',
    )

    map_saver = Node(
        package='slam',
        executable='map_saver',
        name='map_saver',
        parameters=[{
            'use_sim_time': False,
            'map_path': LaunchConfiguration('map_path'),
        }],
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
        wheel_radius_arg,
        wheel_separation_arg,
        robot_state_publisher,
        joint_state_publisher,
        velocity_bridge,
        real_odom,
        lidar_frame_laser,
        lidar_frame_laser_link,
        scan_fix,
        slam_node,
        map_saver,
        rviz,
        teleop,
    ])
