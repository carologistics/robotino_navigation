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

    # -----------------------------------------------------
    # Launch configurations
    # -----------------------------------------------------
    namespace = LaunchConfiguration("namespace")
    map_name = LaunchConfiguration("map")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    params_file = LaunchConfiguration("params_file")
    host_params_file = LaunchConfiguration("host_params_file")
    use_respawn = LaunchConfiguration("use_respawn")
    log_level = LaunchConfiguration("log_level")
    launch_mapserver = LaunchConfiguration("launch_mapserver")

    lifecycle_nodes = ["map_server", "amcl"]

    # -----------------------------------------------------
    # Parameter substitutions
    # -----------------------------------------------------
    param_substitutions = {
        "use_sim_time": use_sim_time,
        "yaml_filename": PathJoinSubstitution(
            [FindPackageShare("robotino_navigation"), "map", map_name]  # automatically builds full path
        ),
    }

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
        (
            PathJoinSubstitution(
                [
                    TextSubstitution(text="/"),
                    namespace,
                    TextSubstitution(text="tf"),
                ]
            ),
            "/tf",
        ),
        (
            PathJoinSubstitution(
                [
                    TextSubstitution(text="/"),
                    namespace,
                    TextSubstitution(text="tf_static"),
                ]
            ),
            "/tf_static",
        ),
        (
            PathJoinSubstitution(
                [
                    TextSubstitution(text="/"),
                    namespace,
                    TextSubstitution(text="map"),
                ]
            ),
            "/map",
        ),
    ]

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
                parameters=[configured_params, configured_host_params],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=remappings,
                condition=IfCondition(launch_mapserver),
                namespace=namespace,
            ),
            Node(
                package="nav2_amcl",
                executable="amcl",
                name="amcl",
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
    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Top-level namespace",
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        "map",
        default_value=PathJoinSubstitution(
            [
                FindPackageShare("robotino_navigation"),
                "map",
                "map_sf_empty.yaml",
            ]
        ),
        description="Full path to map yaml file to load",
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Automatically startup the nav2 stack",
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution(
            [
                FindPackageShare("robotino_navigation"),
                "config",
                "nav2_params.yaml",
            ]
        ),
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )

    declare_host_params_file_cmd = DeclareLaunchArgument(
        "host_params_file",
        default_value=PathJoinSubstitution(
            [FindPackageShare("robotino_navigation"), "config", [LaunchConfiguration("namespace"), "_nav2_params.yaml"]]
        ),
        description="Full path to the host-specific ROS2 parameters file",
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        "use_respawn",
        default_value="false",
        description="Whether to respawn if a node crashes",
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="Log level",
    )

    launch_mapserver_argument = DeclareLaunchArgument(
        "launch_mapserver",
        default_value="true",
        description="Whether to launch map server or not",
    )

    # -----------------------------------------------------
    # Launch description
    # -----------------------------------------------------
    ld = LaunchDescription()

    ld.add_action(SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"))

    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_host_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(launch_mapserver_argument)

    ld.add_action(load_nodes)

    return ld
