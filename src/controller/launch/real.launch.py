"""
real.launch.py — controller stack for the real Puzzlebot MCR2.

Does NOT use ros2_control or controller_manager (those are Gazebo-only).

Starts:
  real_odom    — integrates /wl + /wr velocities → /puzzlebot_controller/odom
                  + odom→base_footprint TF
  vel_smoother — rate-limits acceleration so the powerbank does not see a
                  current spike that reboots the hackerboard. Subscribes to
                  /cmd_vel_in, publishes ramped Twist to /cmd_vel (which the
                  micro_ros agent consumes) and TwistStamped to
                  puzzlebot_controller/cmd_vel (for the sim controller path).

All upstream nodes (navigation, perception, dashboard, teleop) must publish
to /cmd_vel_in — never directly to /cmd_vel.

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
    max_linear_accel_arg = DeclareLaunchArgument(
        'max_linear_accel', default_value='0.6',
        description='Max linear acceleration m/s² — prevents current spikes')
    max_angular_accel_arg = DeclareLaunchArgument(
        'max_angular_accel', default_value='2.0',
        description='Max angular acceleration rad/s²')

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

    # Velocity smoother: sits between all cmd_vel_in publishers and the
    # hackerboard micro_ros agent (/cmd_vel).  Limits acceleration so the
    # power bank doesn't see a current spike that reboots the board.
    vel_smoother = Node(
        package='controller',
        executable='twist_relay',
        name='vel_smoother',
        parameters=[{
            'max_linear_accel':  LaunchConfiguration('max_linear_accel'),
            'max_angular_accel': LaunchConfiguration('max_angular_accel'),
            'rate':             50.0,
            'use_sim_time':     False,
        }],
        output='screen',
    )

    return LaunchDescription([
        wheel_radius_arg,
        wheel_separation_arg,
        max_linear_accel_arg,
        max_angular_accel_arg,
        real_odom,
        vel_smoother,
    ])
