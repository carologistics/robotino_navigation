#!/usr/bin/env python3
# Licensed under MIT. See LICENSE file. Copyright Carologistics.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def generate_launch_description():

    package_share = FindPackageShare("robotino_navigation")

    # -----------------------------------------------------
    # Launch configurations
    # -----------------------------------------------------
    namespace = LaunchConfiguration("namespace")
    map_yaml_file = LaunchConfiguration("map")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    params_file = LaunchConfiguration("params_file")
    LaunchConfiguration("host_params_file")
    use_respawn = LaunchConfiguration("use_respawn")
    log_level = LaunchConfiguration("log_level")
    launch_mapserver = LaunchConfiguration("launch_mapserver")

    lifecycle_nodes = ["map_server"]

    # -----------------------------------------------------
    # Parameter substitutions
    # -----------------------------------------------------
    param_substitutions = {"use_sim_time": use_sim_time, "yaml_filename": map_yaml_file}

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True,
        ),
        allow_substs=True,
    )

    # -----------------------------------------------------
    # Nodes
    # -----------------------------------------------------
    load_nodes = GroupAction(
        actions=[
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
                namespace=namespace,
                condition=IfCondition(launch_mapserver),
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_localization",
                output="screen",
                arguments=["--ros-args", "--log-level", log_level],
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"autostart": autostart},
                    {"node_names": lifecycle_nodes},
                ],
                namespace=namespace,
            ),
        ]
    )

    # -----------------------------------------------------
    # Launch arguments
    # -----------------------------------------------------
    ld = LaunchDescription()

    # Environment
    ld.add_action(SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"))

    ld.add_action(DeclareLaunchArgument("namespace", default_value="", description="Top-level namespace"))
    ld.add_action(
        DeclareLaunchArgument(
            "map",
            default_value=PathJoinSubstitution([package_share, "map", "map_sf_empty.yaml"]),
            description="Full path to map yaml file to load",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "use_sim_time", default_value="false", description="Use simulation (Gazebo) clock if true"
        )
    )
    ld.add_action(
        DeclareLaunchArgument("autostart", default_value="true", description="Automatically startup the nav2 stack")
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
                [package_share, "config", [LaunchConfiguration("namespace"), "_nav2_params.yaml"]]
            ),
            description="Full path to the host-specific ROS2 parameters file to use for all launched nodes",
        )
    )
    ld.add_action(
        DeclareLaunchArgument("use_respawn", default_value="False", description="Whether to respawn if a node crashes")
    )
    ld.add_action(DeclareLaunchArgument("log_level", default_value="info", description="log level"))
    ld.add_action(
        DeclareLaunchArgument("launch_mapserver", default_value="true", description="Whether to launch map server")
    )

    # -----------------------------------------------------
    # Add nodes
    # -----------------------------------------------------
    ld.add_action(load_nodes)

    return ld
