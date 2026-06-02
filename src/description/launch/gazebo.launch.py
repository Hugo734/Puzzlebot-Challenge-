import os
import shutil
from os import pathsep
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable, TimerAction
)
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    puzzlebot_description = get_package_share_directory("description")

    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(puzzlebot_description, "urdf", "puzzlebot_with_lifter.urdf.xacro"),
        description="Absolute path to robot urdf file"
    )

    world_name_arg = DeclareLaunchArgument(
        name="world_name",
        default_value="empty"
    )

    world_path = PathJoinSubstitution([
        puzzlebot_description,
        "worlds",
        PythonExpression(expression=["'", LaunchConfiguration("world_name"), "'", " + '.world'"])
    ])

    model_path = str(Path(puzzlebot_description).parent.resolve())
    model_path += pathsep + os.path.join(puzzlebot_description, 'models')

    ros_lib = "/opt/ros/humble/lib"
    ld_lib = os.environ.get("LD_LIBRARY_PATH", "")
    plugin_path = pathsep.join([ros_lib, ld_lib])

    gz_resource_path = SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", model_path)
    ign_resource_path = SetEnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", model_path)
    # Mesa software GL via GLX — avoids AMD driver GL3Plus crash
    gl_software = SetEnvironmentVariable("LIBGL_ALWAYS_SOFTWARE", "1")

    ros_distro = os.environ["ROS_DISTRO"]
    is_ignition = "True" if ros_distro == "humble" else "False"

    robot_description = ParameterValue(
        Command([
            "xacro ",
            LaunchConfiguration("model"),
            " is_ignition:=", is_ignition
        ]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": True
        }]
    )

    ign_exec = shutil.which("ign") or "ign"

    # Server-only mode (-s): runs physics without the GUI.
    # The GUI can be connected separately with: ign gazebo -g
    # This avoids the SIGINT crash that occurs when the GUI's Qt/ogre2
    # rendering hits a fatal error with the AMD GPU driver.
    # Use a Xvfb virtual display (:99) so Ogre2's GL3Plus render engine uses Mesa
    # software GLX instead of the NVIDIA or AMD hardware driver.  The real display
    # (:0) has NVIDIA 580 drivers installed which ignore LIBGL_ALWAYS_SOFTWARE and
    # the AMD GPU segfaults on GL3Plus buffer upload.
    xvfb = ExecuteProcess(
        cmd=["Xvfb", ":99", "-screen", "0", "1280x1024x24"],
        output="screen",
        on_exit=None,
    )

    gazebo_server = ExecuteProcess(
        cmd=[
            "bash", "-c",
            (
                f"DISPLAY=:99 "
                f"LIBGL_ALWAYS_SOFTWARE=1 "
                f"__GLX_VENDOR_LIBRARY_NAME=mesa "
                f"IGN_IP=127.0.0.1 "
                f"GZ_SIM_SYSTEM_PLUGIN_PATH={plugin_path} "
                f"IGN_GAZEBO_SYSTEM_PLUGIN_PATH={plugin_path} "
                f"GZ_SIM_RESOURCE_PATH={model_path} "
                f"IGN_GAZEBO_RESOURCE_PATH={model_path} "
                f"ruby {ign_exec} gazebo -s -r $0 -v 4 --force-version 6"
            ),
            world_path,
        ],
        output="screen",
        on_exit=None,
    )

    # Wait for the server to be ready before spawning the robot
    gz_spawn_entity = TimerAction(
        period=5.0,
        actions=[Node(
            package="ros_gz_sim",
            executable="create",
            output="screen",
            arguments=["-topic", "robot_description", "-name", "puzzlebot"],
        )]
    )

    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
            "/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU",
            "/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
        ],
        remappings=[("/imu", "/imu/out")],
        additional_env={"IGN_IP": "127.0.0.1"},
    )

    # Fallback joint_state_publisher: publishes zero-position joint states so
    # robot_state_publisher can broadcast TF for all moveable joints.
    # gz_ros2_control v0.7.18 has a persistent service-unresponsive bug that prevents
    # joint_state_broadcaster from spawning; this node ensures /joint_states is always
    # available for visualization even when the controller_manager is broken.
    joint_state_publisher_node = TimerAction(
        period=8.0,
        actions=[Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            parameters=[{"use_sim_time": True}],
        )]
    )

    # Spawn joint_state_broadcaster first, then puzzlebot_controller 6s later.
    # The diff_drive_controller needs command interfaces to be registered by
    # gz_ros2_control before configure — staggering avoids ok=False configure failure.
    jsb_spawner = TimerAction(
        period=12.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster", "--unload-on-kill"],
                output="screen",
            ),
        ]
    )

    diff_drive_spawner = TimerAction(
        period=18.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["puzzlebot_controller", "--unload-on-kill"],
                output="screen",
            ),
        ]
    )

    # Relay /cmd_vel → /puzzlebot_controller/cmd_vel_unstamped
    cmd_vel_relay = Node(
        package="topic_tools",
        executable="relay",
        arguments=["/cmd_vel", "/puzzlebot_controller/cmd_vel_unstamped"],
        output="screen",
    )

    return LaunchDescription([
        model_arg,
        world_name_arg,
        gz_resource_path,
        ign_resource_path,
        gl_software,
        robot_state_publisher_node,
        xvfb,
        gazebo_server,
        gz_spawn_entity,
        gz_ros2_bridge,
        joint_state_publisher_node,
        jsb_spawner,
        diff_drive_spawner,
        cmd_vel_relay,
    ])
