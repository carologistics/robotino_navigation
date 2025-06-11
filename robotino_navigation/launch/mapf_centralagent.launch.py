#!/usr/bin/env python3
# Licensed under MIT. See LICENSE file. Copyright Carologistics.
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
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
    map_yaml_file = LaunchConfiguration("map")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    params_file = LaunchConfiguration("params_file")
    use_respawn = LaunchConfiguration("use_respawn")
    log_level = LaunchConfiguration("log_level")
    launch_mapserver = LaunchConfiguration("launch_mapserver")
    launch_plannerserver = LaunchConfiguration("launch_plannerserver")
    num_of_agents = LaunchConfiguration("number_of_agents")

    lifecycle_nodes = ["mapf_map_server"]
    if launch_plannerserver:
        lifecycle_nodes.append("mapf_planner_server")

    launch_configuration = {}
    for argname, argval in context.launch_configurations.items():
        launch_configuration[argname] = argval

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        "yaml_filename": map_yaml_file,
        "number_of_agents": num_of_agents,
        "use_sim_time": use_sim_time,
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

    # Create the remappings for the nodes
    remappings = [
        ("/" + launch_configuration["namespace"] + "/tf", "/tf"),
        ("/" + launch_configuration["namespace"] + "/tf_static", "/tf_static"),
        ("/" + launch_configuration["namespace"] + "/map", "/map"),
    ]

    os.path.join(bringup_dir, "rviz", "robotino_localization.rviz")

    # Create list of nodes to launch
    load_nodes = GroupAction(
        actions=[
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="mapf_map_server",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=remappings,
                condition=IfCondition(launch_mapserver),
                namespace=namespace,
            ),
            Node(
                package="nav2_mapf_planner",
                executable="mapf_planner_server",
                name="mapf_planner_server",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=remappings,
                condition=IfCondition(launch_plannerserver),
                namespace=namespace,
                # prefix= "konsole -e gdb -ex start --args"
                # prefix= "gdbserver localhost:1200"
                # prefix = "perf record -g -o /home/borse_saurabh/perf.data",
                # prefix=['valgrind --tool=callgrind --dump-instr=yes -v --instr-atstart=no'],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_centralagent",
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

    return [load_nodes]


def generate_launch_description():

    package_dir = get_package_share_directory("robotino_navigation")

    # Declare the launch arguments
    stdout_linebuf_envvar = SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1")

    declare_namespace_cmd = DeclareLaunchArgument("namespace", default_value="mapf", description="Top-level namespace")

    declare_map_yaml_cmd = DeclareLaunchArgument(
        "map",
        default_value=os.path.join(package_dir, "map", "map_sf_sim.yaml"),
        description="Full path to map yaml file to load",
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Automatically startup the nav2 stack",
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=[os.path.join(package_dir, "config/"), "mapf_centralagent.yaml"],
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        "use_respawn",
        default_value="true",
        description="Whether to respawn if a node crashes. Applied when composition is disabled.",
    )

    declare_log_level_cmd = DeclareLaunchArgument("log_level", default_value="info", description="log level")

    declare_launch_mapserver_cmd = DeclareLaunchArgument(
        "launch_mapserver",
        default_value="true",
        description="Wheather to start Rvizor not based on launch environment",
    )

    declare_launch_plannerserver_cmd = DeclareLaunchArgument(
        "launch_plannerserver",
        default_value="true",
        description="Wheather to start Rvizor not based on launch environment",
    )

    declare_launch_num_agents_cmd = DeclareLaunchArgument(
        "number_of_agents",
        default_value="1",
        description="Wheather to start Rvizor not based on launch environment",
    )
    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_launch_mapserver_cmd)
    ld.add_action(declare_launch_plannerserver_cmd)
    ld.add_action(declare_launch_num_agents_cmd)

    # Add the actions to launch all of the localiztion nodes
    ld.add_action(OpaqueFunction(function=launch_nodes_withconfig))

    return ld
