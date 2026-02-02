#!/usr/bin/env python3
# Licensed under MIT. See LICENSE file. Copyright Carologistics.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def generate_launch_description():

    package_share = FindPackageShare("robotino_navigation")

    # -----------------------------------------------------
    # Launch configuration
    # -----------------------------------------------------
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    host_params_file = LaunchConfiguration("host_params_file")

    autostart = True
    lifecycle_nodes = ["collision_monitor"]

    # -----------------------------------------------------
    # Parameter substitutions
    # -----------------------------------------------------
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

    # -----------------------------------------------------
    # Nodes
    # -----------------------------------------------------
    lifecycle_manager_node = Node(
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

    collision_monitor_node = Node(
        package="nav2_collision_monitor",
        executable="collision_monitor",
        output="screen",
        emulate_tty=True,
        parameters=[configured_params, configured_host_params, {"use_sim_time": use_sim_time}],
        namespace=namespace,
    )

    # -----------------------------------------------------
    # Launch arguments
    # -----------------------------------------------------
    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument("namespace", default_value="", description="Top-level namespace"))

    ld.add_action(
        DeclareLaunchArgument("use_sim_time", default_value="true", description="Use simulation (Gazebo) clock if true")
    )

    ld.add_action(
        DeclareLaunchArgument(
            "params_file",
            default_value=PathJoinSubstitution([package_share, "config", "nav2_params.yaml"]),
            description="Full path to the ROS2 parameters file to use for all launched nodes",
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            "host_params_file",
            default_value=PathJoinSubstitution(
                [
                    package_share,
                    "config",
                    [LaunchConfiguration("namespace"), TextSubstitution(text="_nav2_params.yaml")],
                ]
            ),
            description="Full path to the host-specific ROS2 parameters file to use for all launched nodes",
        )
    )

    # -----------------------------------------------------
    # Add nodes
    # -----------------------------------------------------
    ld.add_action(lifecycle_manager_node)
    ld.add_action(collision_monitor_node)

    return ld
