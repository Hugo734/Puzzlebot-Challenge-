"""
EKF Localization Launch  —  fully self-contained inside localization

Modes (pass mode:=sim or mode:=real):

  sim  — kinematic_simulator (C++) converts /cmd_vel -> /wr, /wl
         + ekf_localization  (C++) integrates wheel velocities with EKF
         + robot_state_publisher, joint_state_publisher, RViz, teleop

  real — velocity_bridge    (C++) converts /VelocityEncl, /VelocityEncnR -> /wl, /wr
         + ekf_localization  (C++) integrates wheel velocities with EKF

TF tree: odom -> base_footprint (published by ekf_localization)
         base_footprint -> base_link -> ... (from robot_state_publisher)

Run:
  ros2 launch localization ekf_localization.launch.py mode:=sim
  ros2 launch localization ekf_localization.launch.py mode:=real
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(context, *args, **kwargs):
    mode      = LaunchConfiguration('mode').perform(context)
    open_rviz = LaunchConfiguration('open_rviz').perform(context) == 'true'
    use_icp   = LaunchConfiguration('use_icp').perform(context) == 'true'

    pkg  = get_package_share_directory('localization')
    pkg_desc = get_package_share_directory('description')

    params     = os.path.join(pkg, 'config', 'ekf_params.yaml')
    rviz_cfg   = os.path.join(pkg, 'config', 'ekf.rviz')
    urdf_path  = os.path.join(pkg_desc, 'urdf', 'puzzlebot_with_lifter.urdf.xacro')

    robot_description = ParameterValue(
        Command(['xacro ', urdf_path, ' is_sim:=false is_ignition:=false']),
        value_type=str,
    )

    ekf_node = Node(
        package='localization',
        executable='ekf_localization',
        name='ekf_localization',
        parameters=[params],
        output='screen',
    )

    icp_node = Node(
        package='localization',
        executable='icp_node',
        name='icp_node',
        parameters=[params],
        output='screen',
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': False}],
        output='screen',
    )

    joint_state_publisher = Node(
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

    teleop = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        prefix='xterm -e',
        remappings=[('cmd_vel', '/cmd_vel')],
    )

    if mode == 'sim':
        kinematic_simulator = Node(
            package='localization',
            executable='kinematic_simulator',
            name='kinematic_simulator',
            parameters=[params],
            output='screen',
        )
        nodes = [
            kinematic_simulator,
            ekf_node,
            robot_state_publisher,
            joint_state_publisher,
            teleop,
        ]
        if use_icp:
            nodes.append(icp_node)
        if open_rviz:
            nodes.append(rviz)
        return nodes

    # mode == 'real'
    velocity_bridge = Node(
        package='localization',
        executable='velocity_bridge',
        name='velocity_bridge',
        output='screen',
    )
    nodes = [
        velocity_bridge,
        ekf_node,
        robot_state_publisher,
        joint_state_publisher,
    ]
    if use_icp:
        nodes.append(icp_node)
    if open_rviz:
        nodes.append(rviz)
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'mode',
            default_value='sim',
            description='"sim" uses kinematic_simulator | "real" uses velocity_bridge',
        ),
        DeclareLaunchArgument(
            'open_rviz',
            default_value='true',
            description='Open RViz2 (set false when included from a parent launch)',
        ),
        DeclareLaunchArgument(
            'use_icp',
            default_value='true',
            description='Launch icp_node for scan-to-scan correction. '
                        'Set false when slam_node is running (it handles ICP internally).',
        ),
        OpaqueFunction(function=launch_setup),
    ])
