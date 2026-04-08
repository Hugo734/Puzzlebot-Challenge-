import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    world_name_arg = DeclareLaunchArgument(
        "world_name", default_value="empty"
    )
    use_simple_controller_arg = DeclareLaunchArgument(
        "use_simple_controller", default_value="true"
    )

    # Launch Gazebo simulation with the robot
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory("puzzlebot_description"), "launch"),
            "/gazebo2.launch.py"
        ]),
        launch_arguments={
            "world_name": LaunchConfiguration("world_name"),
        }.items()
    )

    # Launch controller
    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory("puzzlebot_controller"), "launch"),
            "/controller.launch.py"
        ]),
        launch_arguments={
            "use_sim_time": "true",
            "use_simple_controller": LaunchConfiguration("use_simple_controller"),
        }.items()
    )

    return LaunchDescription([
        world_name_arg,
        use_simple_controller_arg,
        gazebo_launch,
        controller_launch,
    ])
