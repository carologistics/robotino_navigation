#!/usr/bin/env python3
# Licensed under MIT. See LICENSE file. Copyright Carologistics.
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.actions import RegisterEventHandler
from launch.actions import Shutdown
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_rviz_with_templated_config(context, *args, **kwargs):
    # Resolve launch arguments
    namespace = LaunchConfiguration("namespace").perform(context)
    launch_rviz = LaunchConfiguration("launch_rviz").perform(context)
    rviz_config = LaunchConfiguration("rviz_config").perform(context)

    if launch_rviz.lower() not in ("true", "1", "yes"):
        return []

    # Read RViz template
    with open(rviz_config, "r") as f:
        content = f.read()

    # Replace placeholder
    content = content.replace("<namespace>", namespace)

    # Write generated RViz config
    tmp_dir = os.path.join("/tmp", "rviz")
    os.makedirs(tmp_dir, exist_ok=True)

    new_rviz_config = os.path.join(
        tmp_dir,
        f"{namespace or 'global'}_nav2config.rviz",
    )

    with open(new_rviz_config, "w") as f:
        f.write(content)

    # RViz node
    start_rviz_cmd = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", new_rviz_config],
        namespace=namespace,
        remappings=[
            (PathJoinSubstitution(["/", namespace, "map"]), "/map"),
            (PathJoinSubstitution(["/", namespace, "tf"]), "/tf"),
            (PathJoinSubstitution(["/", namespace, "tf_static"]), "/tf_static"),
            ("/goal_pose", PathJoinSubstitution(["/", namespace, "goal_pose"])),
            ("/clicked_point", PathJoinSubstitution(["/", namespace, "clicked_point"])),
            ("/initialpose", PathJoinSubstitution(["/", namespace, "initialpose"])),
        ],
    )

    # Shutdown launch when RViz exits
    exit_event_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=start_rviz_cmd,
            on_exit=Shutdown(reason="RViz exited"),
        )
    )

    return [start_rviz_cmd, exit_event_handler]


def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(
        DeclareLaunchArgument(
            "namespace",
            default_value="",
            description="Top-level namespace",
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Whether to start RViz",
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            "rviz_config",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("robotino_navigation"),
                    "rviz",
                    "nav2config.rviz",
                ]
            ),
            description="RViz configuration file",
        )
    )

    ld.add_action(OpaqueFunction(function=launch_rviz_with_templated_config))

    return ld
