"""
robot.launch.py — ON-BOARD (Jetson Nano) half of the distributed real-robot
stack. Pairs with laptop.launch.py.

Design rule (split with laptop.launch.py):
  * The Jetson runs the whole SLAM sensing loop locally, so the LiDAR /scan
    and wheel /odom never cross WiFi before they are matched. That keeps the
    scan↔odom latency budget (< 10 ms, §2) intact.
  * The laptop runs everything else (nav, mission, dashboard, voice, RViz,
    map saver) so the 2 GB Jetson is not loaded — see laptop.launch.py.

Runs here, on the robot:
  controller/real       — velocity_bridge + real_odom + vel_smoother + RSP
                          (wheel odom for SLAM + base→lidar TF + cmd path)
  slam_node             — C++ front-end (GPU MCL) + back-end thread (graph
                          + loop closure + re-mapping). Maps + localises LIVE
                          and publishes /map continuously; nav on the laptop
                          plans over that live grid (no save/reload needed).
  lifting_node          — FPGA lifter control over SPI (Tang Nano 20K on
                          /dev/spidev0.0). Subscribes /lifter_level (UInt8 0-3).
                          Must run on the Jetson — it owns the SPI hardware.

The LiDAR driver is NOT started here — it already runs as its own node,
publishing /scan at 10 Hz. Just make sure that scan's frame_id is `lidar_link`
(the URDF laser frame published by RSP), or add a static transform from the
driver's frame to lidar_link, so slam_node's base→laser TF lookup resolves.

Pre-conditions on the Jetson (started separately, hardware-specific):
  * LiDAR node publishing /scan @ 10 Hz (frame_id = lidar_link).
  * micro-ROS agent bridging the MCR2 hackerboard
      ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyTHS1
    (provides /VelocityEncL, /VelocityEncR, consumes /cmd_vel)
  * Clocks synced with the laptop (chrony); same ROS_DOMAIN_ID on both.
  * Headless — NO RViz here (run it on the laptop).

Live mapping → waypoints → navigation flow (no map save required):
  1. Launch this on the Jetson + laptop.launch.py on the laptop.
  2. Drive (teleop) to build the map — it streams to RViz live.
  3. Define waypoints on that live map from the dashboard / RViz; nav
     hot-reloads them (no restart).
  4. Switch the dashboard to NAVIGATION and send goals — A* plans over the
     live /map. Optionally save the finished map with the map_saver service.

Usage (on the Jetson):
  ros2 launch bringup robot.launch.py
  ros2 launch bringup robot.launch.py scan_time_offset:=0.05
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_controller = get_package_share_directory('controller')
    pkg_slam       = get_package_share_directory('slam')
    pkg_lifting    = get_package_share_directory('lifting')

    slam_params_path    = os.path.join(pkg_slam, 'config', 'slam_params.yaml')
    lifting_params_path = os.path.join(pkg_lifting, 'config', 'lifting_params.yaml')

    # ── Args ──────────────────────────────────────────────────────────
    scan_time_offset_arg = DeclareLaunchArgument(
        'scan_time_offset', default_value='0.10',
        description='Seconds subtracted from /scan stamps to compensate the '
                    'LiDAR capture-to-delivery latency before odom lookup (§2). '
                    'The real RPLidar A1 is ~100-150 ms latent — leaving this 0 '
                    'gives double walls on every turn. Tune 0.05-0.15 to your '
                    'measured driver latency.')
    scan_time_offset = LaunchConfiguration('scan_time_offset')

    # The URDF only publishes the `lidar_link` frame, but the stock RPLidar A1
    # driver stamps scans with frame_id `laser`.  slam_node looks up
    # base_link -> <scan.frame_id> via TF and processes NOTHING until it
    # resolves (silent localisation death).  Bridge laser->lidar_link with an
    # identity static TF so SLAM works regardless of which name the driver uses.
    # Set bridge_laser_frame:=false if you configured the driver to publish
    # frame_id=lidar_link directly (then this bridge is unnecessary).
    bridge_laser_frame_arg = DeclareLaunchArgument(
        'bridge_laser_frame', default_value='true',
        description='Publish an identity lidar_link->laser static TF so SLAM '
                    'resolves the scan frame even if the driver stamps "laser".')

    # Lifter (FPGA over SPI). HAL comes from lifting_params.yaml (hal: spi,
    # spi_bus: 0). Set use_lifter:=false to skip it (e.g. bench runs with no
    # Tang Nano connected, to avoid the spidev open failing).
    use_lifter_arg = DeclareLaunchArgument(
        'use_lifter', default_value='true',
        description='Start the FPGA lifter node (SPI to the Tang Nano 20K).')

    laser_frame_bridge = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='laser_frame_bridge',
        arguments=['0', '0', '0', '0', '0', '0', 'lidar_link', 'laser'],
        condition=IfCondition(LaunchConfiguration('bridge_laser_frame')),
        output='screen',
    )

    # ── 1. Controller / odom / TF (hardware layer, must be local) ──────
    # velocity_bridge + real_odom + vel_smoother + robot_state_publisher.
    # real_odom must run here so /odom is time-synced with the local /scan.
    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_controller, 'launch', 'real.launch.py')))

    # ── 2. SLAM core: live mapping + localisation ──────────────────────
    # Front-end (GPU MCL) + back-end thread (graph/loop-closure/re-mapping).
    # Always maps live and publishes /map; /odom is the local real_odom topic.
    # Short delay so RSP + real_odom are up before the first scan is consumed.
    slam_node = TimerAction(period=1.5, actions=[Node(
        package='slam', executable='slam_node', name='slam_node',
        parameters=[slam_params_path, {
            'use_sim_time':     False,
            'scan_time_offset': scan_time_offset,
        }],
        remappings=[('/odom', '/puzzlebot_controller/odom')],
        output='screen', emulate_tty=True,
    )])

    # ── 3. Lifter (FPGA over SPI) ──────────────────────────────────────
    # Owns /dev/spidev0.0 → Tang Nano 20K. Independent of SLAM, no delay.
    lifting_node = Node(
        package='lifting', executable='lifting_node', name='lifting_node',
        parameters=[lifting_params_path],
        condition=IfCondition(LaunchConfiguration('use_lifter')),
        output='screen',
    )

    return LaunchDescription([
        scan_time_offset_arg,
        bridge_laser_frame_arg,
        use_lifter_arg,
        laser_frame_bridge,
        controller_launch,
        slam_node,
        lifting_node,
    ])
