"""
MCL Full Stack Launch — Gazebo + Odometry + MCL

Starts:
  1. Gazebo simulation (puzzlebot_bringup/simulated_robot.launch.py)
     → provides /scan (LiDAR), /joint_states, odom→base_footprint TF,
       and the simple_velocity_controller that drives the wheels
  2. Kinematic Simulator (inverse kinematics: /cmd_vel → /wr, /wl)
  3. Dead-Reckoning (odometry: /wr, /wl → /odom, for RViz comparison)
  4. MCL Node (Monte Carlo Localization: /scan + /wr, /wl → /particles, /mcl_pose)
  5. RViz2 (visualization of particles, likelihood field, and estimated pose)

Data Flow:
  Teleop (/cmd_vel)
    ├→ twist_relay → simple_controller → Gazebo physics (robot moves)
    │    Gazebo joint_state_broadcaster → simple_controller TF (odom→base_footprint)
    │
    └→ Kinematic Simulator (/wr, /wl)
         ├→ Dead-Reckoning (/odom)   [shown in RViz for comparison]
         └→ MCL Node (/particles, /mcl_pose, /likelihood_map)
              ↓
            RViz2
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_loc = get_package_share_directory('puzzlebot_localization')
    pkg_bringup = get_package_share_directory('puzzlebot_bringup')

    sim_params = os.path.join(pkg_loc, 'config', 'sim_params.yaml')
    rviz_config = os.path.join(pkg_loc, 'config', 'rviz.rviz')

    # 1. Gazebo simulation — provides robot_state_publisher, joint_state_broadcaster,
    #    simple_controller (with odom→base_footprint TF), and twist_relay
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, 'launch', 'simulated_robot.launch.py')
        ),
        launch_arguments={
            'world_name': 'obstacles',
            'use_sim_time': 'true',
        }.items()
    )

    # 2. Kinematic Simulator — converts /cmd_vel → /wr, /wl for dead-reckoning and MCL
    kinematic_simulator = Node(
        package='puzzlebot_localization',
        executable='kinematic_simulator',
        name='kinematic_simulator',
        parameters=[sim_params, {'use_sim_time': True}],
        output='screen',
    )

    # 3. Dead-Reckoning Odometry — integrates /wr, /wl → /odom (for RViz comparison display)
    dead_reckoning = Node(
        package='puzzlebot_localization',
        executable='dead_reckoning',
        name='dead_reckoning',
        parameters=[sim_params, {'use_sim_time': True}],
        output='screen',
    )

    # 4. MCL Node (Monte Carlo Localization)
    mcl_node = Node(
        package='puzzlebot_localization',
        executable='mcl_node',
        name='mcl_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'n_particles':   500,
            'sigma_field_m': 0.2,
            'ray_step':      3,
            'keep_fraction': 0.5,
            'sigma_xy':      0.005,
            'sigma_theta':   0.01,
        }],
    )

    # 5. RViz2
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    )

    return LaunchDescription([
        gazebo_launch,
        kinematic_simulator,
        dead_reckoning,
        mcl_node,
        rviz,
    ])
