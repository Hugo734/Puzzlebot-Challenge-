import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(context, *args, **kwargs):
    mode             = LaunchConfiguration('mode').perform(context)
    world_name       = LaunchConfiguration('world_name').perform(context)
    laser_frame_id   = LaunchConfiguration('laser_frame_id').perform(context)

    pkg_slam = get_package_share_directory('slam')
    slam_params = os.path.join(pkg_slam, 'config', 'slam_params.yaml')
    rviz_cfg    = os.path.join(pkg_slam, 'config', 'slam.rviz')

    # ── Simulation ─────────────────────────────────────────────────────
    if mode == 'sim':
        pkg_bringup = get_package_share_directory('bringup')

        gazebo_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_bringup, 'launch', 'simulated_robot.launch.py')
            ),
            launch_arguments={'world_name': world_name}.items(),
        )

        slam_node = Node(
            package='slam',
            executable='slam_node',
            name='slam_node',
            parameters=[slam_params, {'use_sim_time': True}],
            remappings=[('/odom', '/puzzlebot_controller/odom')],
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
            name='teleop_twist_keyboard',
            output='screen',
            prefix='xterm -e',
            remappings=[('cmd_vel', '/cmd_vel')],
        )

        return [gazebo_launch, slam_node, rviz, teleop]

    # ── Real hardware ─────────────────────────────────────────────────
    pkg_ekf    = get_package_share_directory('localization')
    pkg_desc   = get_package_share_directory('description')
    ekf_params = os.path.join(pkg_ekf, 'config', 'ekf_params.yaml')
    urdf_path  = os.path.join(pkg_desc, 'urdf', 'puzzlebot_mcr2.urdf.xacro')

    robot_description = ParameterValue(
        Command(['xacro ', urdf_path, ' is_sim:=false is_ignition:=false']),
        value_type=str,
    )

    velocity_bridge = Node(
        package='localization',
        executable='velocity_bridge',
        name='velocity_bridge',
        output='screen',
    )

    # Bridge the real LiDAR driver's frame (commonly "laser") to the URDF's
    # `lidar_link`.  Identity transform — the URDF already encodes the 180°
    # mounting yaw on lidar_joint.  Skip if the driver already publishes to
    # lidar_link.
    lidar_tf_bridge = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_frame_bridge',
        arguments=['0', '0', '0', '0', '0', '0', 'lidar_link', laser_frame_id],
        condition=IfCondition(
            PythonExpression(["'", laser_frame_id, "' != 'lidar_link'"])),
    )

    ekf_node = Node(
        package='localization',
        executable='ekf_localization',
        name='ekf_localization',
        parameters=[ekf_params],
        output='screen',
    )

    icp_node = Node(
        package='localization',
        executable='icp_node',
        name='icp_node',
        parameters=[ekf_params],
        output='screen',
    )

    slam_node = Node(
        package='slam',
        executable='slam_node',
        name='slam_node',
        parameters=[slam_params],
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

    teleop_sim = Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_twist_keyboard',
            output='screen',
            prefix='xterm -e',
            remappings=[('cmd_vel', '/cmd_vel')],
    )

    return [
        velocity_bridge,
        lidar_tf_bridge,
        ekf_node,
        icp_node,
        slam_node,
        robot_state_publisher,
        joint_state_publisher,
        rviz,
        teleop_sim,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'mode',
            default_value='sim',
            description='"sim" (Gazebo) | "real" (Jetson hardware)',
        ),
        DeclareLaunchArgument(
            'world_name',
            default_value='obstacles',
            description='Gazebo world name (e.g. "obstacles", "empty")',
        ),
        DeclareLaunchArgument(
            'laser_frame_id',
            default_value='laser',
            description='frame_id used by the real LiDAR driver in /scan headers '
                        '(default: "laser" for RPLidar A1).  Set to "lidar_link" '
                        'to disable the bridging static transform.',
        ),
        OpaqueFunction(function=launch_setup),
    ])
