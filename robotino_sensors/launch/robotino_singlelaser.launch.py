
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

#!/usr/bin/env python

import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
import pathlib
from launch.actions import (LogInfo, RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import  OnProcessStart


def launch_nodes_withconfig(context, *args, **kwargs):

    package_dir = get_package_share_directory('robotino_sensors')
    
    # Declare launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    launch_rviz = LaunchConfiguration('launch_rviz')
    
    launch_configuration = {}
    for argname, argval in context.launch_configurations.items():
        launch_configuration[argname] = argval
    
    # Create a list of nodes to launch
    load_nodes = GroupAction(
        actions=[
            
        Node(
            package='sick_scan_xd',
            executable='sick_generic_caller',
            name='sick_scan',
            output='screen',
            namespace = namespace,
            parameters= [os.path.join(package_dir, 'config', 'laserSens_config.yaml'),
                         {'frame_id':launch_configuration['namespace']+'/laser_link',
                          'hostname':'192.168.2.63',#169.254.4.93
                          'port': '2112'}],
        ), 
        
        # Spawn Rviz2 node for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(package_dir,'rviz','lasersens_rvizconfig.rviz')],
            output='screen',
            condition = IfCondition(launch_rviz),
        ), 
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0.5', '0.5', '0.5', '0', '0', '0', 'map',launch_configuration['namespace']+'/laser_link'],
        ),
        
        ])
    return[load_nodes]

def generate_launch_description():
    package_dir = get_package_share_directory('robotino_sensors')
    
    # Declare launch configuration variables
    declare_namespace_argument = DeclareLaunchArgument(
        'namespace', default_value='', 
        description='Top-level namespace')

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation clock if true')
    
    declare_launch_rviz_argument = DeclareLaunchArgument(
        'launch_rviz',
        default_value='true', 
        description= 'Wheather to start Rviz or not based on launch environment')
    
    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_argument)
    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_launch_rviz_argument)

    # Add the actions to launch all nodes
    ld.add_action(OpaqueFunction(function=launch_nodes_withconfig))
    
    return ld