#!/usr/bin/env python3
# Licensed under MIT. See LICENSE file. Copyright Carologistics.
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get the launch directory
    package_dir = get_package_share_directory("robotino_navigation")
    launch_dir = os.path.join(package_dir, "launch")

    # Declare the launch arguments
    stdout_linebuf_envvar = SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1")

    declare_launch_mps_map_gen_cmd = DeclareLaunchArgument(
        "launch_mps_map_gen",
        default_value="false",
        description="Weather to launch mps_map_gen or not",
    )
    declare_launch_map_filter_cmd = DeclareLaunchArgument(
        "launch_map_filter",
        default_value="true",
        description="Whether to launch map server or not",
    )
    launch_map_filter = LaunchConfiguration("launch_map_filter")

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Declare the launch options
    ld.add_action(declare_launch_mps_map_gen_cmd)
    ld.add_action(declare_launch_map_filter_cmd)

    # Conditionally include another launch file
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare("mps_map_gen"), "launch", "mps_map_gen.launch.py"])
            ),
            condition=IfCondition(LaunchConfiguration("launch_mps_map_gen")),
        )
    )
    # robotino_localization
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, "robotino_localization.launch.py")),
            launch_arguments={}.items(),
        )
    )

    # robotino_navigation
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, "robotino_navigation.launch.py")),
            launch_arguments={}.items(),
        )
    )

    # robotino_costmapfilter
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, "robotino_costmap_filter.launch.py")),
            launch_arguments={}.items(),
            condition=IfCondition(launch_map_filter),
        )
    )

    # robotino_rviz
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, "robotino_rviz.launch.py")),
            launch_arguments={}.items(),
        )
    )
    return ld
