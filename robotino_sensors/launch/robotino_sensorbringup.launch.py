#!/usr/bin/env python
# Licensed under MIT. See LICENSE file. Copyright Carologistics.
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def launch_nodes_withconfig(context, *args, **kwargs):

    # Declare launch configuration variables
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_rviz = LaunchConfiguration("launch_rviz")
    launch_ekf = LaunchConfiguration("launch_ekf")

    launch_configuration = {}
    for argname, argval in context.launch_configurations.items():
        launch_configuration[argname] = argval  #

    # launch robotinobase controllers with individual namespaces
    load_launchfiles = GroupAction(
        actions=[
            # Launch robotinobase1 controller
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("robotino_sensors"),
                                "launch",
                                "robotino_ekffusion.launch.py",
                            ]
                        )
                    ]
                ),
                launch_arguments={
                    "namespace": namespace,
                    "use_sim_time": use_sim_time,
                    "launch_rviz": launch_rviz,
                }.items(),
            ),
            # Launch robotinobase1 controller
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("robotino_sensors"),
                                "launch",
                                "robotino_integratedlaser.launch.py",
                            ]
                        )
                    ]
                ),
                launch_arguments={
                    "namespace": namespace,
                    "use_sim_time": use_sim_time,
                    "launch_ekf": launch_ekf,
                }.items(),
            ),
        ]
    )

    return [load_launchfiles]


def generate_launch_description():
    get_package_share_directory("robotino_sensors")

    # Declare launch configuration variables
    declare_namespace_argument = DeclareLaunchArgument("namespace", default_value="", description="Top-level namespace")

    declare_use_sim_time_argument = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation clock if true",
    )

    declare_launch_rviz_argument = DeclareLaunchArgument(
        "launch_rviz",
        default_value="false",
        description="Wheather to start Rvizor not based on launch environment",
    )

    declare_launch_ekf_argument = DeclareLaunchArgument(
        "launch_ekf",
        default_value="true",
        description="Wheather to start ekf fusion for odometry or not based on launch environment",
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_argument)
    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_launch_rviz_argument)
    ld.add_action(declare_launch_ekf_argument)

    # Add the actions to launch webots, controllers and rviz
    ld.add_action(OpaqueFunction(function=launch_nodes_withconfig))

    return ld
