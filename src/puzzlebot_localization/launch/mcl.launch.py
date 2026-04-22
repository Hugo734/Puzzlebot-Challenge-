from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument('n_particles',   default_value='500'),
        DeclareLaunchArgument('sigma_field_m', default_value='0.2'),
        DeclareLaunchArgument('ray_step',      default_value='5'),
        DeclareLaunchArgument('keep_fraction', default_value='0.5'),
        DeclareLaunchArgument('sigma_xy',      default_value='0.02'),
        DeclareLaunchArgument('sigma_theta',   default_value='0.05'),

        Node(
            package='puzzlebot_localization',
            executable='mcl_node',
            name='mcl_node',
            output='screen',
            parameters=[{
                'n_particles':   LaunchConfiguration('n_particles'),
                'sigma_field_m': LaunchConfiguration('sigma_field_m'),
                'ray_step':      LaunchConfiguration('ray_step'),
                'keep_fraction': LaunchConfiguration('keep_fraction'),
                'sigma_xy':      LaunchConfiguration('sigma_xy'),
                'sigma_theta':   LaunchConfiguration('sigma_theta'),
            }],
        ),
    ])
