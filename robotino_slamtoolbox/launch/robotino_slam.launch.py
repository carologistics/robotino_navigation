# Licensed under MIT. See LICENSE file. Copyright Carologistics.
import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from robotino_utils import find_file


def launch_nodes_withconfig(context, *args, **kwargs):

    # Declare launch configuration variables
    LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_rviz = LaunchConfiguration("launch_rviz")
    slam_config = LaunchConfiguration("slam_config")
    host_config = LaunchConfiguration("host_config")
    rviz_config = LaunchConfiguration("rviz_config")
    package_dir = get_package_share_directory("robotino_slamtoolbox")

    launch_configuration = {}
    for argname, argval in context.launch_configurations.items():
        launch_configuration[argname] = argval  #

    rviz_config_file = find_file(rviz_config.perform(context), [package_dir + "/rviz/"])
    with open(rviz_config_file, "r") as file:
        rviz_template_content = file.read()
    namespace = context.launch_configurations["namespace"]
    replaced_rviz_config = rviz_template_content.replace("<namespace>", namespace)
    slam_config_file = find_file(slam_config.perform(context), [package_dir + "/config/"])
    if slam_config_file is None:
        print("Can not find host config %s, abort", slam_config.perform(context))
        sys.exit(1)
    host_config_file = find_file(host_config.perform(context), [package_dir + "/config/"])
    if host_config_file is None:
        print("Can not find %s, abort!", host_config.perform(context))
        sys.exit(1)

    # Create a list of nodes to launch
    load_nodes = GroupAction(
        actions=[
            # Initialize SLAM Toolbox node in asynchronous mode
            Node(
                parameters=[
                    slam_config_file,
                    host_config_file,
                    {
                        "use_sim_time": use_sim_time,
                        "odom_frame": launch_configuration["namespace"] + "/odom",
                        "base_frame": launch_configuration["namespace"] + "/base_link",
                        "scan_topic": launch_configuration["namespace"] + "/scan",
                    },
                ],
                package="slam_toolbox",
                executable="async_slam_toolbox_node",
                name="slam_toolbox",
                output="screen",
                # namespace=namespace,
            ),
            # Initialize rviz2
            Node(
                package="rviz2",
                executable="rviz2",
                output="screen",
                arguments=["--display-config=" + replaced_rviz_config],
                # namespace=namespace,
                parameters=[{"use_sim_time": use_sim_time}],
                condition=IfCondition(launch_rviz),
            ),
        ]
    )
    return [load_nodes]


def generate_launch_description():
    package_dir = get_package_share_directory("robotino_slamtoolbox")

    # Declare launch configuration variables
    declare_namespace_argument = DeclareLaunchArgument("namespace", default_value="", description="Top-level namespace")

    declare_use_sim_time_argument = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="Use simulation clock if true"
    )

    declare_launch_rviz_argument = DeclareLaunchArgument(
        "launch_rviz",
        default_value="true",
        description="Wheather to start Rvizor not based on launch environment",
    )
    declare_rviz_config_argument = DeclareLaunchArgument(
        "rviz_config",
        default_value=[os.path.join(package_dir, "rviz/"), "slam.rviz"],
        description="Full path to the RVIZ config file to use",
    )

    declare_slam_config_argument = DeclareLaunchArgument(
        "slam_config",
        default_value=os.path.join(package_dir, "config", "slam_params.yaml"),
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

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_argument)
    ld.add_action(declare_rviz_config_argument)
    ld.add_action(declare_slam_config_argument)
    ld.add_action(declare_host_config_argument)
    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_launch_rviz_argument)

    # Add the actions to launch all nodes
    ld.add_action(OpaqueFunction(function=launch_nodes_withconfig))

    return ld
