#!/usr/bin/env python3
# MIT License
#
# Copyright (c) 2024
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.actions import SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def launch_nodes_withconfig(context, *args, **kwargs):

    # Get the launch directory
    bringup_dir = get_package_share_directory("robotino_navigation")
    launch_dir = os.path.join(bringup_dir, "launch")

    # Create the launch configuration variables
    namespace = LaunchConfiguration("namespace")
    launch_rviz = LaunchConfiguration("launch_rviz")
    use_composition = LaunchConfiguration("use_composition")
    map_yaml_file = LaunchConfiguration("map")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    autostart = LaunchConfiguration("autostart")
    use_respawn = LaunchConfiguration("use_respawn")
    launch_mapserver = LaunchConfiguration("launch_mapserver")
    launch_nav2rviz = LaunchConfiguration("launch_nav2rviz")
    rviz_config = LaunchConfiguration("rviz_config")

    launch_configuration = {}
    for argname, argval in context.launch_configurations.items():
        launch_configuration[argname] = argval

    params_file = os.path.join(bringup_dir, "config", launch_configuration["namespace"] + "_nav2_params.yaml")

    rviz_config = os.path.join(bringup_dir, "rviz", launch_configuration["namespace"] + "_nav2config.rviz")

    # Specify the actions
    bringup_cmd_group = GroupAction(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(launch_dir, "robotino_localization.launch.py")),
                launch_arguments={
                    "namespace": namespace,
                    "map": map_yaml_file,
                    "use_sim_time": use_sim_time,
                    "autostart": autostart,
                    "params_file": params_file,
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
                    "use_composition": use_composition,
                    "use_respawn": use_respawn,
                }.items(),
            ),
            # IncludeLaunchDescription(
            #    PythonLaunchDescriptionSource(os.path.join(launch_dir, 'robotino_collisionmonitor.launch.py')),
            #    launch_arguments={'namespace': namespace,
            #                      'use_sim_time': use_sim_time,
            #                      'params_file': params_file,
            #                      }.items()),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(launch_dir, "robotino_rviz.launch.py")),
                launch_arguments={
                    "namespace": namespace,
                    "launch_rviz": launch_nav2rviz,
                    "rviz_config": rviz_config,
                }.items(),
            ),
        ]
    )

    return [bringup_cmd_group]


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory("robotino_navigation")

    # Declare the launch arguments
    stdout_linebuf_envvar = SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1")

    declare_namespace_cmd = DeclareLaunchArgument("namespace", default_value="", description="Top-level namespace")

    declare_launch_rviz_cmd = DeclareLaunchArgument(
        "launch_rviz",
        default_value="false",
        description="Weather to use namespace or not",
    )

    declare_use_composition_cmd = DeclareLaunchArgument(
        "use_composition",
        default_value="False",
        description="Weather to use composition or not",
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        "map",
        default_value=os.path.join(bringup_dir, "map", "map_sf.yaml"),
        description="Full path to map yaml file to load",
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value="",
        # default_value=os.path.join(bringup_dir, 'config', 'robotinobase3_nav2_params.yaml'),
        description="Full path to the ROS2 parameters file to use for all launched nodes",
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

    declare_launch_nav2rviz_cmd = DeclareLaunchArgument(
        "launch_nav2rviz",
        default_value="false",
        description="whether to launch rviz or not",
    )

    declare_rvizconfig_cmd = DeclareLaunchArgument(
        "rviz_config",
        default_value="",
        description="Full path to the RVIZ config file to use for all launched nodes",
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_launch_rviz_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_launchmapserver_cmd)
    ld.add_action(declare_launch_nav2rviz_cmd)
    ld.add_action(declare_rvizconfig_cmd)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(OpaqueFunction(function=launch_nodes_withconfig))

    return ld
