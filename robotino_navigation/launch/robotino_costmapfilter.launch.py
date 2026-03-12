#!/usr/bin/env python3
# Licensed under MIT. See LICENSE file. Copyright Carologistics.
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import LogInfo
from launch.actions import OpaqueFunction
from launch.actions import SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def launch_nodes_withconfig(context, *args, **kwargs):

    bringup_dir = get_package_share_directory("robotino_navigation")

    # Create the launch configuration variables
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    params_file = LaunchConfiguration("params_file")
    host_params_file = LaunchConfiguration("host_params_file")
    use_respawn = LaunchConfiguration("use_respawn")
    log_level = LaunchConfiguration("log_level")
    launch_map_filter = LaunchConfiguration("launch_map_filter")
    filter_mask_yaml = LaunchConfiguration("filter_mask_yaml")
    goal_poses_file = LaunchConfiguration("goal_poses_file")

    lifecycle_nodes = ["costmap_filter_info_server", "filter_mask_server"]

    # Create our own temporary YAML files that include substitutions
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

    launch_configuration = {}
    for argname, argval in context.launch_configurations.items():
        launch_configuration[argname] = argval

    # Create the remappings for the nodes
    remappings = [
        ("/" + launch_configuration["namespace"] + "/tf", "/tf"),
        ("/" + launch_configuration["namespace"] + "/tf_static", "/tf_static"),
        ("/" + launch_configuration["namespace"] + "/map", "/map"),
    ]

    # Parse goal poses from YAML file and create static transform publishers
    static_transform_publishers = []
    goal_poses_file_path = goal_poses_file.perform(context)

    if os.path.exists(goal_poses_file_path):
        static_transforms = {}
        with open(goal_poses_file_path, "r") as file:
            transforms = yaml.safe_load(file)
            # Navigate YAML structure: /**/ros__parameters/static_transforms
            for key in "/**|ros__parameters|static_transforms".split("|"):
                transforms = transforms.get(key, {})
            for entity, transform in transforms.items():
                if not isinstance(static_transforms.get(entity), dict):
                    static_transforms[entity] = {}
                for key in ["translation", "rotation", "parent_frame_id", "child_frame_id"]:
                    if key in transform:
                        static_transforms[entity][key] = transform[key]

        # Create static transform publisher nodes for each goal pose
        for transform_name, values in static_transforms.items():
            translation = values.get("translation", None)
            rotation = values.get("rotation", None)
            frame_id = values.get("parent_frame_id", None)
            child_frame_id = values.get("child_frame_id", None)

            if None in [translation, rotation, frame_id, child_frame_id]:
                static_transform_publishers.append(
                    LogInfo(msg=f"[WARN] Missing key(s) in transform '{transform_name}'. Skipping...")
                )
                continue

            static_transform_publisher_node = Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                output="screen",
                namespace=namespace,
                name="tf_goalpose_" + transform_name,
                arguments=[
                    "--x",
                    str(translation[0]),
                    "--y",
                    str(translation[1]),
                    "--z",
                    str(translation[2]),
                    "--yaw",
                    str(rotation[0]),
                    "--pitch",
                    str(rotation[1]),
                    "--roll",
                    str(rotation[2]),
                    "--frame-id",
                    frame_id,
                    "--child-frame-id",
                    child_frame_id,
                ],
            )
            static_transform_publishers.append(static_transform_publisher_node)
    else:
        static_transform_publishers.append(
            LogInfo(msg=f"[WARN] Goal poses file not found: {goal_poses_file_path}")
        )

    # Create list of nodes to launch
    load_nodes = GroupAction(
        actions=static_transform_publishers
        + [
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="filter_mask_server",
                condition=IfCondition(use_sim_time and launch_map_filter),
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
                condition=IfCondition(launch_map_filter),
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
                condition=IfCondition(launch_map_filter),
            ),
        ]
    )

    return [load_nodes]


def generate_launch_description():

    package_dir = get_package_share_directory("robotino_navigation")

    # Declare the launch arguments
    stdout_linebuf_envvar = SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1")
    stdout_color_envvar = SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1")

    declare_namespace_cmd = DeclareLaunchArgument("namespace", default_value="", description="Top-level namespace")

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
        default_value=[os.path.join(package_dir, "config/"), "nav2_params.yaml"],
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )

    declare_host_params_file_cmd = DeclareLaunchArgument(
        "host_params_file",
        default_value=[os.path.join(package_dir, "config/"), LaunchConfiguration("namespace"), "_nav2_params.yaml"],
        description="Full path to the host-specific ROS2 parameters file to use for all launched nodes",
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        "use_respawn",
        default_value="False",
        description="Whether to respawn if a node crashes. Applied when composition is disabled.",
    )

    declare_log_level_cmd = DeclareLaunchArgument("log_level", default_value="debug", description="log level")

    launch_mapserver_argument = DeclareLaunchArgument(
        "launch_map_filter",
        default_value="true",
        description="Wheather to launch map server or not",
    )

    declare_filter_mask_yaml_cmd = DeclareLaunchArgument(
        "filter_mask_yaml",
        default_value=os.path.join(package_dir, "map", "filter_mask.yaml"),
        description="Full path to filter mask yaml file to load",
    )

    declare_goal_poses_file_cmd = DeclareLaunchArgument(
        "goal_poses_file",
        default_value=os.path.join(package_dir, "config", "goal_pses.yaml"),
        description="Full path to goal poses YAML file for TF publishing",
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(stdout_color_envvar)

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(launch_mapserver_argument)
    ld.add_action(declare_host_params_file_cmd)
    ld.add_action(declare_filter_mask_yaml_cmd)
    ld.add_action(declare_goal_poses_file_cmd)

    # Add the actions to launch all of the localiztion nodes
    ld.add_action(OpaqueFunction(function=launch_nodes_withconfig))

    return ld
