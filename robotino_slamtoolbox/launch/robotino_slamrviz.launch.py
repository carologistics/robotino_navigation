# Licensed under MIT. See LICENSE file. Copyright Carologistics.
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_nodes_withconfig(context, *args, **kwargs):

    # Declare launch configuration variables
    LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_rviz = LaunchConfiguration("launch_rviz")

    launch_configuration = {}
    for argname, argval in context.launch_configurations.items():
        launch_configuration[argname] = argval  #

    slam_rviz_config = os.path.join(
        get_package_share_directory("robotino_slamtoolbox"),
        "rviz",
        launch_configuration["namespace"] + "_slam.rviz",
    )

    # Create a list of nodes to launch
    load_nodes = GroupAction(
        actions=[
            # Initialize rviz2
            Node(
                package="rviz2",
                executable="rviz2",
                output="screen",
                arguments=["--display-config=" + slam_rviz_config],
                # namespace=namespace,
                parameters=[{"use_sim_time": use_sim_time}],
                condition=IfCondition(launch_rviz),
            )
        ]
    )
    return [load_nodes]


def generate_launch_description():

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

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_argument)
    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_launch_rviz_argument)

    # Add the actions to launch all nodes
    ld.add_action(OpaqueFunction(function=launch_nodes_withconfig))

    return ld
