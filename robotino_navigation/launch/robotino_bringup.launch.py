#!/usr/bin/env python3
# Licensed under MIT. See LICENSE file. Copyright Carologistics.
import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.actions import SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from robotino_utils import find_file


def launch_nodes_withconfig(context, *args, **kwargs):

    # Get the launch directory
    bringup_dir = get_package_share_directory("robotino_navigation")
    launch_dir = os.path.join(bringup_dir, "launch")

    # Create the launch configuration variables
    namespace = LaunchConfiguration("namespace")
    launch_rviz = LaunchConfiguration("launch_rviz")
    launch_mps_map_gen = LaunchConfiguration("launch_mps_map_gen")
    use_composition = LaunchConfiguration("use_composition")
    input_map_yaml_file = LaunchConfiguration("map")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    use_respawn = LaunchConfiguration("use_respawn")
    launch_mapserver = LaunchConfiguration("launch_mapserver")
    launch_mapfilter = LaunchConfiguration("launch_mapfilter")
    launch_nav2rviz = LaunchConfiguration("launch_nav2rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    input_params_file = LaunchConfiguration("params_file")
    input_host_params_file = LaunchConfiguration("host_params_file")
    team_name = LaunchConfiguration("team_name")

    launch_configuration = {}
    for argname, argval in context.launch_configurations.items():
        launch_configuration[argname] = argval

    map_yaml_file = find_file(input_map_yaml_file.perform(context), [bringup_dir + "/map/"])
    if map_yaml_file is None:
        print("Can not find %s, abort!", input_map_yaml_file.perform(context))
        sys.exit(1)
    params_file = find_file(input_params_file.perform(context), [bringup_dir + "/config/"])
    if params_file is None:
        print("Can not find %s, abort!", input_params_file.perform(context))
        sys.exit(1)
    host_params_file = find_file(input_host_params_file.perform(context), [bringup_dir + "/config/"])
    if host_params_file is None:
        print("Can not find %s, abort!", input_host_params_file.perform(context))
        sys.exit(1)

    launch_mps_map_gen_value = launch_mps_map_gen.perform(context).lower() in ["true", "1", "t", "y", "yes"]
    if launch_mps_map_gen_value:
        mps_map_gen_dir = get_package_share_directory("mps_map_gen")

    # Specify the actions
    actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, "robotino_localization.launch.py")),
            launch_arguments={
                "namespace": namespace,
                "map": map_yaml_file,
                "use_sim_time": use_sim_time,
                "autostart": autostart,
                "params_file": params_file,
                "host_params_file": host_params_file,
                "use_composition": use_composition,
                "use_respawn": use_respawn,
                "launch_mapserver": launch_mapserver,
                "launch_rviz": launch_rviz,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, "robotino_navigation.launch.py")),
            launch_arguments={
                "namespace": namespace,
                "use_sim_time": use_sim_time,
                "autostart": autostart,
                "params_file": params_file,
                "host_params_file": host_params_file,
                "use_composition": use_composition,
                "use_respawn": use_respawn,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, "robotino_costmapfilter.launch.py")),
            launch_arguments={
                "namespace": namespace,
                "use_sim_time": use_sim_time,
                "autostart": autostart,
                "params_file": params_file,
                "host_params_file": host_params_file,
                "use_respawn": use_respawn,
                "launch_map_filter": launch_mapfilter,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, "robotino_rviz.launch.py")),
            launch_arguments={
                "namespace": namespace,
                "launch_rviz": launch_nav2rviz,
                "rviz_config": rviz_config,
            }.items(),
        ),
    ]
    if launch_mps_map_gen_value:
        actions.extend(
            [
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(os.path.join(mps_map_gen_dir, "mps_map_gen.launch.py")),
                    launch_arguments={
                        "namespace": namespace,
                        "use_sim_time": use_sim_time,
                        "get_data_from_refbox": "true",
                        "publish_wait_pos": "true",
                        "peer_address": "172.26.255.255",
                        "team_name": team_name,
                        "recv_port_cyan": "4441",
                        "recv_port_magenta": "4442",
                        "recv_port_public": "4444",
                        "crypto_key": "randomkey",
                        "map_client": "/map_server/map",
                    }.items(),
                )
            ]
        )
    bringup_cmd_group = GroupAction(actions)

    return [bringup_cmd_group]


def generate_launch_description():
    # Get the launch directory
    package_dir = get_package_share_directory("robotino_navigation")

    # Declare the launch arguments
    stdout_linebuf_envvar = SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1")
    stdout_color_envvar = SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1")
    stdout_format_envar = SetEnvironmentVariable(
        "RCUTILS_CONSOLE_OUTPUT_FORMAT", "[{severity}] [{time}] [{name}] [{function_name}] [{line}]: {message}"
    )

    declare_namespace_cmd = DeclareLaunchArgument("namespace", default_value="", description="Top-level namespace")

    declare_launch_rviz_cmd = DeclareLaunchArgument(
        "launch_rviz",
        default_value="false",
        description="Weather to launch rviz or not",
    )

    declare_launch_mps_map_gen_cmd = DeclareLaunchArgument(
        "launch_mps_map_gen",
        default_value="false",
        description="Weather to launch mps_map_gen or not",
    )

    declare_use_composition_cmd = DeclareLaunchArgument(
        "use_composition",
        default_value="False",
        description="Weather to use composition or not",
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        "map",
        default_value=os.path.join(package_dir, "map", "map_RC2025.yaml"),
        description="Full path to map yaml file to load",
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=[os.path.join(package_dir, "config/"), "nav2_params.yaml"],
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )

    declare_host_params_file_cmd = DeclareLaunchArgument(
        "host_params_file",
        default_value=[
            os.path.join(package_dir, "config/"),
            LaunchConfiguration("namespace"),
            "_nav2_params",
            ".yaml",
        ],
        description="Full path to the host-specific ROS2 parameters file to use for all launched nodes",
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Automatically startup the nav2 stack",
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        "use_respawn",
        default_value="False",
        description="Whether to respawn if a node crashes. Applied when composition is disabled.",
    )

    declare_log_level_cmd = DeclareLaunchArgument("log_level", default_value="info", description="log level")

    declare_launchmapserver_cmd = DeclareLaunchArgument(
        "launch_mapserver",
        default_value="true",
        description="whether to launch map server or not",
    )

    declare_launchmapfilter_cmd = DeclareLaunchArgument(
        "launch_mapfilter",
        default_value="true",
        description="whether to launch keepout filer or not",
    )

    declare_launch_nav2rviz_cmd = DeclareLaunchArgument(
        "launch_nav2rviz",
        default_value="false",
        description="whether to launch rviz or not",
    )

    declare_rvizconfig_cmd = DeclareLaunchArgument(
        "rviz_config",
        default_value=[os.path.join(package_dir, "rviz/"), "nav2config.rviz"],
        description="Full path to the RVIZ config file to use for all launched nodes",
    )

    declare_team_name_cmd = DeclareLaunchArgument(
        "team_name",
        default_value="Carologistics",
        description="Which team name registered for the RefBox (important for mps_map_gen)",
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(stdout_color_envvar)
    ld.add_action(stdout_format_envar)

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_launch_rviz_cmd)
    ld.add_action(declare_launch_mps_map_gen_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_host_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_launchmapserver_cmd)
    ld.add_action(declare_launchmapfilter_cmd)
    ld.add_action(declare_launch_nav2rviz_cmd)
    ld.add_action(declare_rvizconfig_cmd)
    ld.add_action(declare_team_name_cmd)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(OpaqueFunction(function=launch_nodes_withconfig))

    return ld
