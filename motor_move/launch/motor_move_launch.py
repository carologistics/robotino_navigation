# Licensed under MIT. See LICENSE file. Copyright Carologistics.
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration  # noqa: F401
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    package_dir = get_package_share_directory("motor_move")
    config = os.path.join(package_dir, "config", "motor_move.yaml")

    # Launch configurations
    use_sim_time = LaunchConfiguration("use_sim_time")
    namespace = LaunchConfiguration("namespace")

    # Parameter substitutions dictionary
    param_substitutions = {"use_sim_time": use_sim_time}

    # Create temporary YAML with substitutions
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=config,
            root_key=namespace,  # assumes namespace is defined in YAML structure under nodes
            param_rewrites=param_substitutions,
            convert_types=True,
        ),
        allow_substs=True,
    )

    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="false", description="Use simulation (Gazebo) clock if true"
    )

    namespace_arg = DeclareLaunchArgument(
        "namespace", default_value="/", description="Namespace for the MotorMove node."  # default to the root namespace
    )

    # Node definition
    motor_move_node = Node(
        package="motor_move",
        executable="motor_move",
        namespace=namespace,
        name="motor_move",
        output="screen",
        parameters=[configured_params],
    )

    return LaunchDescription([use_sim_time_arg, namespace_arg, motor_move_node])
