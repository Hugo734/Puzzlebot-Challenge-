import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_loc = get_package_share_directory('puzzlebot_localization')
    pkg_bringup = get_package_share_directory('puzzlebot_bringup')

    rviz_config = os.path.join(pkg_loc, 'config', 'rviz.rviz')

    # Gazebo simulation with robot
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, 'launch', 'simulated_robot.launch.py')
        ),
        launch_arguments={
            'world_name': 'obstacles',
            'use_sim_time': 'true',
        }.items()
    )

    # MCL Node
    mcl_node = Node(
        package='puzzlebot_localization',
        executable='mcl_node',
        name='mcl_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'n_particles':   LaunchConfiguration('n_particles'),
            'sigma_field_m': LaunchConfiguration('sigma_field_m'),
            'ray_step':      LaunchConfiguration('ray_step'),
            'keep_fraction': LaunchConfiguration('keep_fraction'),
            'sigma_xy':      LaunchConfiguration('sigma_xy'),
            'sigma_theta':   LaunchConfiguration('sigma_theta'),
        }],
    )

    # RViz2
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    )

    # Teleop (optional keyboard control)
    teleop = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        prefix='xterm -e',
        remappings=[('cmd_vel', '/cmd_vel')],
    )

    return LaunchDescription([
        # Launch arguments for MCL tuning
        DeclareLaunchArgument('n_particles',   default_value='500'),
        DeclareLaunchArgument('sigma_field_m', default_value='0.2'),
        DeclareLaunchArgument('ray_step',      default_value='5'),
        DeclareLaunchArgument('keep_fraction', default_value='0.5'),
        DeclareLaunchArgument('sigma_xy',      default_value='0.02'),
        DeclareLaunchArgument('sigma_theta',   default_value='0.05'),

        # Launch all components
        gazebo_launch,
        mcl_node,
        rviz,
        teleop,
    ])
