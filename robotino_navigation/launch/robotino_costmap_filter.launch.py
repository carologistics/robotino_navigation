#!/usr/bin/env python3
# Licensed under MIT. See LICENSE file. Copyright Carologistics.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import SetEnvironmentVariable
from launch.conditions import IfCondition
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
    # Launch configurations
    # -----------------------------------------------------
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    params_file = LaunchConfiguration("params_file")
    host_params_file = LaunchConfiguration("host_params_file")
    use_respawn = LaunchConfiguration("use_respawn")
    log_level = LaunchConfiguration("log_level")
    launch_map_filter = LaunchConfiguration("launch_map_filter")
    filter_mask_yaml = LaunchConfiguration("filter_mask_yaml")

    lifecycle_nodes = ["costmap_filter_info_server", "filter_mask_server"]

    # -----------------------------------------------------
    # Parameter substitutions
    # -----------------------------------------------------
    param_substitutions = {"use_sim_time": use_sim_time, "yaml_filename": filter_mask_yaml}

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
    # Remappings
    # -----------------------------------------------------
    remappings = [
        (PathJoinSubstitution([TextSubstitution(text="/"), namespace, TextSubstitution(text="tf")]), "/tf"),
        (
            PathJoinSubstitution([TextSubstitution(text="/"), namespace, TextSubstitution(text="tf_static")]),
            "/tf_static",
        ),
        (PathJoinSubstitution([TextSubstitution(text="/"), namespace, TextSubstitution(text="map")]), "/map"),
    ]

    # -----------------------------------------------------
    # Nodes
    # -----------------------------------------------------
    load_nodes = GroupAction(
        actions=[
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="filter_mask_server",
                output="screen",
                respawn=use_respawn,
                emulate_tty=True,
                parameters=[configured_params, configured_host_params],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=remappings,
                namespace=namespace,
            ),
            Node(
                package="nav2_map_server",
                executable="costmap_filter_info_server",
                name="costmap_filter_info_server",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params, configured_host_params],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=remappings,
                namespace=namespace,
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_costmap_filters",
                output="screen",
                arguments=["--ros-args", "--log-level", log_level],
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"autostart": autostart},
                    {"node_names": lifecycle_nodes},
                ],
                namespace=namespace,
            ),
        ],
        condition=IfCondition(launch_map_filter),  # <-- condition applies to all children
    )

    # -----------------------------------------------------
    # Launch arguments
    # -----------------------------------------------------
    ld = LaunchDescription()

    ld.add_action(SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"))

    ld.add_action(DeclareLaunchArgument("namespace", default_value="", description="Top-level namespace"))
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
                [
                    package_share,
                    "config",
                    [LaunchConfiguration("namespace"), TextSubstitution(text="_nav2_params.yaml")],
                ]
            ),
            description="Full path to the host-specific ROS2 parameters file to use for all launched nodes",
        )
    )
    ld.add_action(
        DeclareLaunchArgument("use_respawn", default_value="False", description="Whether to respawn if a node crashes")
    )
    ld.add_action(DeclareLaunchArgument("log_level", default_value="info", description="log level"))
    ld.add_action(
        DeclareLaunchArgument(
            "launch_map_filter", default_value="true", description="Whether to launch map filter nodes"
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "filter_mask_yaml",
            default_value=PathJoinSubstitution([package_share, "map", "filter_mask.yaml"]),
            description="Full path to filter mask yaml file to load",
        )
    )

    # -----------------------------------------------------
    # Add nodes
    # -----------------------------------------------------
    ld.add_action(load_nodes)

    return ld
