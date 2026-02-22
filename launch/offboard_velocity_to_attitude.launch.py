from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    default_params = PathJoinSubstitution(
        [
            FindPackageShare("vsp_to_qsp"),
            "config",
            "offboard_velocity_to_attitude.params.yaml",
        ]
    )

    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_params,
        description="Path to the parameter file for velocity->attitude node.",
    )

    node = Node(
        package="vsp_to_qsp",
        executable="velocity_to_attitude_node",
        name="offboard_velocity_to_attitude_node",
        output="screen",
        parameters=[LaunchConfiguration("params_file")],
    )

    return LaunchDescription([params_file_arg, node])
