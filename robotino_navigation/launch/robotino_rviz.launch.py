# Author: Saurabh Borse(saurabh.borse@alumni.fh-aachen.de)

#  MIT License
#  Copyright (c) 2023 Saurabh Borse
#  Permission is hereby granted, free of charge, to any person obtaining a copy
#  of this software and associated documentation files (the "Software"), to deal
#  in the Software without restriction, including without limitation the rights
#  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
#  copies of the Software, and to permit persons to whom the Software is
#  furnished to do so, subject to the following conditions:

#  The above copyright notice and this permission notice shall be included in all
#  copies or substantial portions of the Software.

#  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
#  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
#  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
#  SOFTWARE.

#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import ReplaceString

def launch_nodes_withconfig(context, *args, **kwargs):
    # Get the launch directory
    bringup_dir = get_package_share_directory('robotino_navigation')

    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    launch_rviz = LaunchConfiguration('launch_rviz')
    rviz_config = LaunchConfiguration('rviz_config')

    launch_configuration = {}
    for argname, argval in context.launch_configurations.items():
        launch_configuration[argname] = argval

    rviz_config_dir = os.path.join(bringup_dir,'rviz', launch_configuration['rviz_config'])

    start_namespaced_rviz_cmd = Node(
        condition=IfCondition(launch_rviz),
        package='rviz2',
        executable='rviz2',
        namespace=namespace,
        arguments=['-d', rviz_config_dir],
        output='screen',
        remappings=[('/'+launch_configuration['namespace']+'/map', '/map'),
                    ('/'+launch_configuration['namespace']+'/tf', '/tf'),
                    ('/'+launch_configuration['namespace']+'/tf_static', 'tf_static'),
                    ('/goal_pose', '/'+launch_configuration['namespace']+'/goal_pose'),
                    ('/clicked_point', '/'+launch_configuration['namespace']+'/clicked_point'),
                    ('/initialpose', '/'+launch_configuration['namespace']+'/initialpose')])

    exit_event_handler_namespaced = RegisterEventHandler(
        condition=IfCondition(launch_rviz),
        event_handler=OnProcessExit(
            target_action=start_namespaced_rviz_cmd,
            on_exit=EmitEvent(event=Shutdown(reason='rviz exited'))))

    return [start_namespaced_rviz_cmd,
            exit_event_handler_namespaced]

def generate_launch_description():

    bringup_dir = get_package_share_directory('robotino_navigation')

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description=('Top-level namespace. The value will be used to replace the '
                     '<robot_namespace> keyword on the rviz config file.'))

    declare_launch_rviz_cmd = DeclareLaunchArgument(
        'launch_rviz',
        default_value='true',
        description='Whether to start rviz or not')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(bringup_dir, 'rviz', 'robotinobase4_nav2config.rviz'),
        description='Full path to the RVIZ config file to use')


    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_launch_rviz_cmd)
    ld.add_action(declare_rviz_config_file_cmd)

    ld.add_action(OpaqueFunction(function=launch_nodes_withconfig))

    return ld
