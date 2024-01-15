# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

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
    bringup_dir = get_package_share_directory('robotino3_navigation')
    
    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    rviz_config_file = LaunchConfiguration('rviz_config')

    launch_configuration = {}
    for argname, argval in context.launch_configurations.items():
        launch_configuration[argname] = argval
        
    rviz_config_dir = os.path.join(bringup_dir,'rviz', launch_configuration['rviz_config'])
    
    start_namespaced_rviz_cmd = Node(
        condition=IfCondition(use_namespace),
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
        condition=IfCondition(use_namespace),
        event_handler=OnProcessExit(
            target_action=start_namespaced_rviz_cmd,
            on_exit=EmitEvent(event=Shutdown(reason='rviz exited'))))
        
    return [start_namespaced_rviz_cmd, 
            exit_event_handler_namespaced]

def generate_launch_description():
    
    bringup_dir = get_package_share_directory('robotino3_navigation')
   
    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description=('Top-level namespace. The value will be used to replace the '
                     '<robot_namespace> keyword on the rviz config file.'))

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='true',
        description='Whether to apply a namespace to the navigation stack')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(bringup_dir, 'rviz', 'robotinobase1_nav2config.rviz'),
        description='Full path to the RVIZ config file to use')


    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_rviz_config_file_cmd)

    ld.add_action(OpaqueFunction(function=launch_nodes_withconfig))
    
    return ld
