
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
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction, GroupAction
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.conditions import IfCondition


def launch_nodes_withconfig(context, *args, **kwargs):

    # Declare launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    launch_rviz = LaunchConfiguration('launch_rviz')

    launch_configuration = {}
    for argname, argval in context.launch_configurations.items():
        launch_configuration[argname] = argval#

    slam_rviz_config = os.path.join(get_package_share_directory('robotino_slamtoolbox'), 'rviz', launch_configuration['namespace']+'_slam.rviz')

    # Create a list of nodes to launch
    load_nodes = GroupAction(
        actions=[

        # Initialize SLAM Toolbox node in asynchronous mode
        Node(
            parameters=[
                get_package_share_directory("robotino_slamtoolbox") + '/config/slam_params.yaml',
                {'use_sim_time': use_sim_time,
                'odom_frame': launch_configuration['namespace']+'/odom',
                'base_frame': launch_configuration['namespace']+'/base_link',
                'scan_topic': launch_configuration['namespace']+'/scan',}
                ],
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            #namespace=namespace,
        ),

        # Initialize rviz2
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['--display-config=' + slam_rviz_config],
            #namespace=namespace,
            parameters=[{'use_sim_time': use_sim_time}],
            condition = IfCondition(launch_rviz)
        )

        ])
    return[load_nodes]

def generate_launch_description():

    # Declare launch configuration variables
    declare_namespace_argument = DeclareLaunchArgument(
        'namespace', default_value='',
        description='Top-level namespace')

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation clock if true')

    declare_launch_rviz_argument = DeclareLaunchArgument(
        'launch_rviz',
        default_value='true',
        description= 'Wheather to start Rvizor not based on launch environment')

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_argument)
    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_launch_rviz_argument)

    # Add the actions to launch all nodes
    ld.add_action(OpaqueFunction(function=launch_nodes_withconfig))

    return ld
