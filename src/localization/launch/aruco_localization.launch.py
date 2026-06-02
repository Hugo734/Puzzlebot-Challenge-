"""
ArUco + EKF Localization Launch
================================
Brings up the full localization stack with ArUco absolute-pose corrections
in addition to (or instead of) LiDAR ICP corrections.

Modes (mode:=sim | mode:=real):

  sim  — kinematic_simulator converts /cmd_vel → /wr /wl
  real — velocity_bridge converts /VelocityEncl /VelocityEncnR → /wl /wr

Nodes started:
  ├─ velocity_bridge OR kinematic_simulator  (mode-dependent)
  ├─ ekf_localization   (C++)   — wheel odometry + EKF prediction/update
  ├─ aruco_localizer    (Python) — camera → ArUco → /pose_measurement
  ├─ icp_node           (C++)   — optional, disable when ArUco is enough
  ├─ robot_state_publisher
  └─ rviz2              (optional)

Run:
  ros2 launch localization aruco_localization.launch.py mode:=real
  ros2 launch localization aruco_localization.launch.py mode:=sim open_rviz:=true
  ros2 launch localization aruco_localization.launch.py mode:=real use_icp:=true
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(context, *args, **kwargs):
    mode       = LaunchConfiguration('mode').perform(context)
    open_rviz  = LaunchConfiguration('open_rviz').perform(context) == 'true'
    use_icp    = LaunchConfiguration('use_icp').perform(context) == 'true'

    pkg_loc  = get_package_share_directory('localization')
    pkg_desc = get_package_share_directory('description')

    ekf_params   = os.path.join(pkg_loc, 'config', 'ekf_params.yaml')
    aruco_params = os.path.join(pkg_loc, 'config', 'aruco_map.yaml')
    rviz_cfg     = os.path.join(pkg_loc, 'config', 'ekf.rviz')
    urdf_path    = os.path.join(pkg_desc, 'urdf', 'puzzlebot_with_lifter.urdf.xacro')

    robot_description = ParameterValue(
        Command(['xacro ', urdf_path, ' is_sim:=false is_ignition:=false']),
        value_type=str,
    )

    ekf_node = Node(
        package='localization',
        executable='ekf_localization',
        name='ekf_localization',
        parameters=[ekf_params],
        output='screen',
    )

    aruco_node = Node(
        package='perception',
        executable='aruco_localizer',
        name='aruco_localizer',
        parameters=[aruco_params],
        output='screen',
    )

    icp_node = Node(
        package='localization',
        executable='icp_node',
        name='icp_node',
        parameters=[ekf_params],
        output='screen',
    )

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': False}],
        output='screen',
    )

    joint_state_pub = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_cfg],
        output='screen',
    )

    # ── Common nodes always started ──────────────────────────────────────
    nodes = [robot_state_pub, joint_state_pub, ekf_node, aruco_node]

    if use_icp:
        nodes.append(icp_node)

    if open_rviz:
        nodes.append(rviz)

    # ── Mode-specific node ───────────────────────────────────────────────
    if mode == 'sim':
        nodes.append(Node(
            package='localization',
            executable='kinematic_simulator',
            name='kinematic_simulator',
            parameters=[ekf_params],
            output='screen',
        ))
    else:
        nodes.append(Node(
            package='localization',
            executable='velocity_bridge',
            name='velocity_bridge',
            output='screen',
        ))

    return nodes


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            'mode',
            default_value='real',
            description='"real" uses velocity_bridge | "sim" uses kinematic_simulator',
        ),
        DeclareLaunchArgument(
            'open_rviz',
            default_value='false',
            description='Open RViz2 (set true for desktop debugging)',
        ),
        DeclareLaunchArgument(
            'use_icp',
            default_value='false',
            description='Also run icp_node for LiDAR scan-matching corrections. '
                        'Useful as a complement to ArUco when markers are not visible.',
        ),
        OpaqueFunction(function=launch_setup),
    ])
