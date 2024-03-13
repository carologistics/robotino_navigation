#!/usr/bin/env python
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
import sys

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from robotino_utils import find_file


def launch_nodes_withconfig(context, *args, **kwargs):

    package_dir = get_package_share_directory("robotino_sensors")

    # Declare launch configuration variables
    namespace = LaunchConfiguration("namespace")
    host_config = LaunchConfiguration("host_config")
    sensor_config = LaunchConfiguration("sensor_config")
    LaunchConfiguration("use_sim_time")
    launch_rviz = LaunchConfiguration("launch_rviz")
    host_config_file = find_file(host_config.perform(context), [package_dir + "/config/"])
    if host_config_file is None:
        print("Can not find %s, abort!", host_config.perform(context))
        sys.exit(1)
    sensor_config_file = find_file(sensor_config.perform(context), [package_dir + "/config/"])
    if sensor_config_file is None:
        print("Can not find host config %s, abort", sensor_config.perform(context))
        sys.exit(1)

    launch_configuration = {}
    for argname, argval in context.launch_configurations.items():
        launch_configuration[argname] = argval
    static_transform_publishers = []
    static_transforms = {}
    with open(sensor_config_file, "r") as file:
        transforms = yaml.safe_load(file)
        for key in "/**|ros__parameters|static_transforms".split("|"):
            transforms = transforms.get(key, {})
        for entity, transform in transforms.items():
            if not isinstance(static_transforms.get(entity), dict):
                static_transforms[entity] = {}
            for key in ["translation", "rotation", "parent_frame_id", "child_frame_id"]:
                if key in transform:
                    static_transforms[entity][key] = transform[key]
    with open(host_config_file, "r") as file:
        transforms = yaml.safe_load(file)
        for key in "/**|ros__parameters|static_transforms".split("|"):
            transforms = transforms.get(key, {})
        for entity, transform in transforms.items():
            if not isinstance(static_transforms.get(entity), dict):
                static_transforms[entity] = {}
            for key in ["translation", "rotation", "parent_frame_id", "child_frame_id"]:
                if key in transform:
                    static_transforms[entity][key] = transform[key]
    print(static_transforms)
    for transform, values in static_transforms.items():
        translation = values.get("translation", None)
        rotation = values.get("rotation", None)
        frame_id = values.get("parent_frame_id", None)
        child_frame_id = values.get("child_frame_id", None)

        if None in [translation, rotation, frame_id, child_frame_id]:
            static_transform_publishers.append(LogInfo(msg="[WARN] Missing key(s) in transform. Skipping..."))
            continue
        static_transform_publisher_node = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="screen",
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

    namespace_frontlaser = launch_configuration["namespace"] + "/front"
    namespace_rearlaser = launch_configuration["namespace"] + "/back"
    load_nodes = GroupAction(
        actions=static_transform_publishers
        + [
            Node(
                package="sick_scan_xd",
                executable="sick_generic_caller",
                name="front_laser",
                output="screen",
                namespace=namespace_frontlaser,
                parameters=[
                    sensor_config_file,
                    host_config_file,
                ],
                remappings=[("/cloud", namespace_frontlaser + "/cloud")],
            ),
            Node(
                package="sick_scan_xd",
                executable="sick_generic_caller",
                name="rear_laser",
                output="screen",
                namespace=namespace_rearlaser,
                parameters=[
                    sensor_config_file,
                    host_config_file,
                ],
                remappings=[("/cloud", namespace_rearlaser + "/cloud")],
            ),
            # Launch Integrate laserscan launch file
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("laser_scan_integrator"),
                                "launch",
                                "integrated_scan.launch.py",
                            ]
                        )
                    ]
                ),
                launch_arguments={
                    "namespace": namespace,
                }.items(),
            ),
            # Spawn Rviz2 node for visualization
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=[
                    "-d",
                    os.path.join(package_dir, "rviz", "integratedlaser_rvizconfig.rviz"),
                ],
                output="screen",
                condition=IfCondition(launch_rviz),
            ),
        ]
    )
    return [load_nodes]


def generate_launch_description():
    package_dir = get_package_share_directory("robotino_sensors")

    # Declare launch configuration variables
    declare_namespace_argument = DeclareLaunchArgument("namespace", default_value="", description="Top-level namespace")

    declare_sensor_config_argument = DeclareLaunchArgument(
        "sensor_config",
        default_value=os.path.join(package_dir, "config", "laser_config.yaml"),
        description="Full path to laser config file to load",
    )

    declare_host_config_argument = DeclareLaunchArgument(
        "host_config",
        default_value=[
            os.path.join(package_dir, "config/"),
            LaunchConfiguration("namespace"),
            ".yaml",
        ],
        description="path to host-specific configs",
    )

    declare_use_sim_time_argument = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation clock if true",
    )

    declare_launch_rviz_argument = DeclareLaunchArgument(
        "launch_rviz",
        default_value="false",
        description="Wheather to start Rviz or not based on launch environment",
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_argument)
    ld.add_action(declare_sensor_config_argument)
    ld.add_action(declare_host_config_argument)
    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_launch_rviz_argument)

    # Add the actions to launch all nodes
    ld.add_action(OpaqueFunction(function=launch_nodes_withconfig))

    return ld
