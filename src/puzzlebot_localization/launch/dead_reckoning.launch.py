"""
Dead-Reckoning Odometry Launch

Starts:
  - kinematic_simulator   /cmd_vel  ->  /wr, /wl
  - dead_reckoning        /wr, /wl  ->  /odom
  - tf_broadcaster        /odom     ->  TF odom->base_footprint
  - robot_state_publisher URDF      ->  TF base_footprint->base_link->...
  - joint_state_publisher           ->  /joint_states (zero positions)
  - rviz2                           ->  visualization
  - teleop_twist_keyboard           ->  manual velocity input

TF tree: odom -> base_footprint -> base_link -> wheels / sensors
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_loc = get_package_share_directory('puzzlebot_localization')
    pkg_desc = get_package_share_directory('puzzlebot_description')

    params = os.path.join(pkg_loc, 'config', 'sim_params.yaml')
    rviz_config = os.path.join(pkg_loc, 'config', 'rviz.rviz')
    urdf_path = os.path.join(pkg_desc, 'urdf', 'puzzlebot.urdf.xacro')

    # Robot description (strip Gazebo plugins via is_sim:=false)
    robot_description = ParameterValue(
        Command(['xacro ', urdf_path, ' is_sim:=false is_ignition:=false']),
        value_type=str,
    )

    kinematic_simulator = Node(
        package='puzzlebot_localization',
        executable='kinematic_simulator',
        name='kinematic_simulator',
        parameters=[params],
        output='screen',
    )

    dead_reckoning = Node(
        package='puzzlebot_localization',
        executable='dead_reckoning',
        name='dead_reckoning',
        parameters=[params],
        output='screen',
    )

    tf_broadcaster = Node(
        package='puzzlebot_localization',
        executable='tf_broadcaster',
        name='tf_broadcaster',
        parameters=[params],
        output='screen',
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False,
        }],
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
        arguments=['-d', rviz_config],
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

    return LaunchDescription([
        kinematic_simulator,
        dead_reckoning,
        tf_broadcaster,
        robot_state_publisher,
        joint_state_publisher,
        rviz,
        teleop,
    ])
