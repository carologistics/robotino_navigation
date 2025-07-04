#!/usr/bin/env python3
# Licensed under MIT. See LICENSE file. Copyright Carologistics.
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import EmitEvent
from launch.actions import OpaqueFunction
from launch.actions import RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_nodes_withconfig(context, *args, **kwargs):
    # Get the launch directory
    bringup_dir = get_package_share_directory("robotino_navigation")

    # Create the launch configuration variables
    namespace = LaunchConfiguration("namespace")
    launch_rviz = LaunchConfiguration("launch_rviz")
    LaunchConfiguration("rviz_config")

    launch_configuration = {}
    for argname, argval in context.launch_configurations.items():
        launch_configuration[argname] = argval

    rviz_config_file = launch_configuration["rviz_config"]
    # Replace placeholders with actual values
    with open(rviz_config_file, "r") as file:
        rviz_template_content = file.read()
    namespace = context.launch_configurations["namespace"]
    replaced_content = rviz_template_content.replace("<namespace>", namespace)

    # Write the modified content to a new RVIZ2 configuration file
    new_rviz_config_path = os.path.join(bringup_dir, "rviz", namespace + "_nav2config.rviz")
    with open(new_rviz_config_path, "w") as file:
        file.write(replaced_content)

    start_namespaced_rviz_cmd = Node(
        condition=IfCondition(launch_rviz),
        package="rviz2",
        executable="rviz2",
        namespace=namespace,
        arguments=["-d", new_rviz_config_path],
        output="screen",
        parameters=[{"namespace", launch_configuration["namespace"]}],
        remappings=[
            # /{namespace}/map -> /map (shared global map)
            # Note: TF topics are NOT remapped - each robot maintains its own TF tree
            ("goal_pose", "/goal_pose"),
            ("clicked_point","clicked_point"),
            ("initialpose", "initialpose"),
        ],
    )

    exit_event_handler_namespaced = RegisterEventHandler(
        condition=IfCondition(launch_rviz),
        event_handler=OnProcessExit(
            target_action=start_namespaced_rviz_cmd,
            on_exit=EmitEvent(event=Shutdown(reason="rviz exited")),
        ),
    )

    return [start_namespaced_rviz_cmd, exit_event_handler_namespaced]


def generate_launch_description():

    package_dir = get_package_share_directory("robotino_navigation")

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description=(
            "Top-level namespace. The value will be used to replace the "
            "<robot_namespace> keyword on the rviz config file."
        ),
    )

    declare_launch_rviz_cmd = DeclareLaunchArgument(
        "launch_rviz", default_value="true", description="Whether to start rviz or not"
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        "rviz_config",
        default_value=[os.path.join(package_dir, "rviz/"), "nav2config.rviz"],
        description="Full path to the RVIZ config file to use for all launched nodes",
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_launch_rviz_cmd)
    ld.add_action(declare_rviz_config_file_cmd)

    ld.add_action(OpaqueFunction(function=launch_nodes_withconfig))

    return ld
