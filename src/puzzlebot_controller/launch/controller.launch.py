from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="true"
    )
    use_simple_controller_arg = DeclareLaunchArgument(
        "use_simple_controller", default_value="true"
    )
    wheel_radius_arg = DeclareLaunchArgument(
        "wheel_radius", default_value="0.05"
    )
    wheel_separation_arg = DeclareLaunchArgument(
        "wheel_separation", default_value="0.19"
    )

    use_simple_controller = LaunchConfiguration("use_simple_controller")
    wheel_radius = LaunchConfiguration("wheel_radius")
    wheel_separation = LaunchConfiguration("wheel_separation")

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    # Standard diff_drive_controller (when not using simple controller)
    wheel_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "puzzlebot_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        condition=UnlessCondition(use_simple_controller),
    )

    # Simple velocity controller + custom controller node
    simple_controller_group = GroupAction(
        condition=IfCondition(use_simple_controller),
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "simple_velocity_controller",
                    "--controller-manager",
                    "/controller_manager",
                ],
            ),
            Node(
                package="puzzlebot_controller",
                executable="simple_controller",
                parameters=[{
                    "wheel_radius": wheel_radius,
                    "wheel_separation": wheel_separation,
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                }],
            ),
        ],
    )

    # Twist relay (converts Twist -> TwistStamped)
    twist_relay_node = Node(
        package="puzzlebot_controller",
        executable="twist_relay",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        condition=IfCondition(use_simple_controller),
    )

    return LaunchDescription([
        use_sim_time_arg,
        use_simple_controller_arg,
        wheel_radius_arg,
        wheel_separation_arg,
        joint_state_broadcaster_spawner,
        wheel_controller_spawner,
        simple_controller_group,
        twist_relay_node,
    ])
