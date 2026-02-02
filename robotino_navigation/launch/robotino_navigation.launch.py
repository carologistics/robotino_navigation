#!/usr/bin/env python3
# Licensed under MIT. See LICENSE file. Copyright Carologistics.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def generate_launch_description():

    # -----------------------------------------------------
    # Package share
    # -----------------------------------------------------
    robotino_share = FindPackageShare("robotino_navigation")

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

    # -----------------------------------------------------
    # Lifecycle nodes
    # -----------------------------------------------------
    lifecycle_nodes = [
        "controller_server",
        "smoother_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
        "waypoint_follower",
        "velocity_smoother",
    ]

    # -----------------------------------------------------
    # Remappings (substitution-safe)
    # -----------------------------------------------------
    remappings = [
        (PathJoinSubstitution(["/", namespace, "tf"]), "/tf"),
        (PathJoinSubstitution(["/", namespace, "tf_static"]), "/tf_static"),
        (PathJoinSubstitution(["/", namespace, "map"]), "/map"),
    ]

    # -----------------------------------------------------
    # Parameter rewriting
    # -----------------------------------------------------
    param_substitutions = {
        "use_sim_time": use_sim_time,
        "autostart": autostart,
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
    # Navigation nodes
    # -----------------------------------------------------
    nav_nodes = GroupAction(
        actions=[
            Node(
                package="nav2_smoother",
                executable="smoother_server",
                name="smoother_server",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params, configured_host_params],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=remappings,
                namespace=namespace,
            ),
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params, configured_host_params],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=remappings,
                namespace=namespace,
            ),
            Node(
                package="nav2_controller",
                executable="controller_server",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params, configured_host_params],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=remappings
                + [
                    ("/robotinobase1/cmd_vel", "/robotinobase1/cmd_vel_nav"),
                ],
                namespace=namespace,
            ),
            Node(
                package="nav2_behaviors",
                executable="behavior_server",
                name="behavior_server",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params, configured_host_params],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=remappings,
                namespace=namespace,
            ),
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params, configured_host_params],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=remappings,
                namespace=namespace,
            ),
            Node(
                package="nav2_waypoint_follower",
                executable="waypoint_follower",
                name="waypoint_follower",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params, configured_host_params],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=remappings,
                namespace=namespace,
            ),
            Node(
                package="nav2_velocity_smoother",
                executable="velocity_smoother",
                name="velocity_smoother",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params, configured_host_params],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=remappings
                + [
                    ("/robotinobase1/cmd_vel", "/robotinobase1/cmd_vel_nav"),
                    ("/robotinobase1/cmd_vel_smoothed", "/robotinobase1/cmd_vel"),
                ],
                namespace=namespace,
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
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
    # Launch description
    # -----------------------------------------------------
    ld = LaunchDescription()

    ld.add_action(SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"))

    ld.add_action(DeclareLaunchArgument("namespace", default_value="", description="Top-level namespace"))
    ld.add_action(DeclareLaunchArgument("use_sim_time", default_value="true"))
    ld.add_action(
        DeclareLaunchArgument(
            "params_file",
            default_value=PathJoinSubstitution([robotino_share, "config", "nav2_params.yaml"]),
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "host_params_file",
            default_value=PathJoinSubstitution(
                [robotino_share, "config", [LaunchConfiguration("namespace"), "_nav2_params.yaml"]]
            ),
        )
    )
    ld.add_action(DeclareLaunchArgument("autostart", default_value="true"))
    ld.add_action(DeclareLaunchArgument("use_respawn", default_value="True"))
    ld.add_action(DeclareLaunchArgument("log_level", default_value="info"))

    ld.add_action(nav_nodes)

    return ld
