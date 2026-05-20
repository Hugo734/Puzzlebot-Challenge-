"""
real.launch.py — controller stack for the real Puzzlebot MCR2.

Does NOT use ros2_control or controller_manager (those are Gazebo-only).

Starts:
  real_odom    — integrates /wl + /wr velocities → /puzzlebot_controller/odom
                  + odom→base_footprint TF
  twist_relay  — converts /cmd_vel (Twist) → puzzlebot_controller/cmd_vel
                  (TwistStamped) so simple_controller's velCallback fires
                  (needed only if something downstream reads that topic)

Run alongside:
  ros2 launch slam mapping_real.launch.py   (provides velocity_bridge, RSP, SLAM)
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    wheel_radius_arg = DeclareLaunchArgument(
        'wheel_radius', default_value='0.05',
        description='Wheel radius in metres')
    wheel_separation_arg = DeclareLaunchArgument(
        'wheel_separation', default_value='0.19',
        description='Wheel centre-to-centre distance in metres')

    real_odom = Node(
        package='controller',
        executable='real_odom',
        name='real_odom',
        parameters=[{
            'wheel_radius':    LaunchConfiguration('wheel_radius'),
            'wheel_separation': LaunchConfiguration('wheel_separation'),
            'use_sim_time':    False,
        }],
        output='screen',
    )

    return LaunchDescription([
        wheel_radius_arg,
        wheel_separation_arg,
        real_odom,
    ])
