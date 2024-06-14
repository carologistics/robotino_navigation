#!/usr/bin/env python3
# Licensed under MIT. See LICENSE file. Copyright Carologistics.
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def launch_nodes_withconfig(context, *args, **kwargs):

    # Create the launch configuration variables
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    host_params_file = LaunchConfiguration("host_params_file")

    # Constant parameters
    lifecycle_nodes = ["collision_monitor"]
    autostart = True

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {"use_sim_time": use_sim_time}

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True,
        ),
        allow_substs=True,
    )
    configured_host_params = ParameterFile(
        RewrittenYaml(
            source_file=host_params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True,
        ),
        allow_substs=True,
    )

    launch_configuration = {}
    for argname, argval in context.launch_configurations.items():
        launch_configuration[argname] = argval

    # Nodes launching commands
    start_lifecycle_manager_cmd = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager",
        output="screen",
        emulate_tty=True,
        parameters=[
            {"use_sim_time": use_sim_time},
            {"autostart": autostart},
            {"node_names": lifecycle_nodes},
        ],
        namespace=namespace,
    )

    start_collision_monitor_cmd = Node(
        package="nav2_collision_monitor",
        executable="collision_monitor",
        output="screen",
        emulate_tty=True,
        parameters=[configured_params, configured_host_params, {"use_sim_time": use_sim_time}],
        namespace=namespace,
    )

    return [start_lifecycle_manager_cmd, start_collision_monitor_cmd]


def generate_launch_description():
    # Environment
    package_dir = get_package_share_directory("robotino_navigation")

    # 2. Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument("namespace", default_value="", description="Top-level namespace")

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=[os.path.join(package_dir, "config/"), "nav2_params.yaml"],
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )

    declare_host_params_file_cmd = DeclareLaunchArgument(
        "host_params_file",
        default_value=[os.path.join(package_dir, "config/"), LaunchConfiguration("namespace"), "_nav2_params.yaml"],
        description="Full path to the host-specific ROS2 parameters file to use for all launched nodes",
    )

    ld = LaunchDescription()

    # Launch arguments
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_host_params_file_cmd)

    # Node launching commands
    ld.add_action(OpaqueFunction(function=launch_nodes_withconfig))

    return ld
