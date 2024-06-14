# Licensed under MIT. See LICENSE file. Copyright Carologistics.
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_nodes_withconfig(context, *args, **kwargs):

    # Get the launch directory
    bringup_dir = get_package_share_directory("robotino3_sensors")

    # Create the launch configuration variables
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_ukf = LaunchConfiguration("launch_ukf")

    launch_configuration = {}
    for argname, argval in context.launch_configurations.items():
        launch_configuration[argname] = argval

    ukf_node = Node(
        condition=IfCondition(launch_ukf),
        package="robot_localization",
        executable="ukf_node",
        name="ukf_filter_node",
        output="screen",
        parameters=[
            os.path.join(bringup_dir, "config", "ukf.yaml"),
            {
                "use_sim_time": use_sim_time,
                "map_frame": "map",
                "odom_frame": launch_configuration["namespace"] + "/odom",
                "base_link_frame": launch_configuration["namespace"] + "/base_link",
                "world_frame": launch_configuration["namespace"] + "/odom",
                "odom0": "/" + launch_configuration["namespace"] + "/odom",
                "imu0": "/" + launch_configuration["namespace"] + "/imu",
            },
        ],
        namespace=namespace,
        remappings=[
            (
                "/" + launch_configuration["namespace"] + "/odometry/filtered",
                "/" + launch_configuration["namespace"] + "/odom_filtered",
            ),
        ],
    )

    return [ukf_node]


def generate_launch_description():

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace", default_value="robotinobase1", description="Top-level namespace"
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )

    declare_launch_ukf_cmd = DeclareLaunchArgument(
        "launch_ukf",
        default_value="false",
        description="Weather to use ekf_fusion for odometry",
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_launch_ukf_cmd)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(OpaqueFunction(function=launch_nodes_withconfig))

    return ld
