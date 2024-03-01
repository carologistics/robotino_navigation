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

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, LogInfo,
                            RegisterEventHandler)
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import (AndSubstitution, LaunchConfiguration,
                                  NotSubstitution)
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node


def launch_nodes_withconfig(context, *args, **kwargs):

    # Declare launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    launch_rviz = LaunchConfiguration('launch_rviz')
    autostart = LaunchConfiguration('autostart')
    use_lifecycle_manager = LaunchConfiguration("use_lifecycle_manager")
    slam_params_file = LaunchConfiguration('slam_params_file')

    launch_configuration = {}
    for argname, argval in context.launch_configurations.items():
        launch_configuration[argname] = argval#

    slam_rviz_config = os.path.join(get_package_share_directory('robotino_slamtoolbox'), 'rviz', launch_configuration['namespace']+'_slam.rviz')


    # Initialize SLAM Toolbox node in asynchronous mode
    start_async_slam_toolbox_node = LifecycleNode(
        parameters=[
            slam_params_file,
                {'use_sim_time': use_sim_time,
                'odom_frame': launch_configuration['namespace']+'/odom',
                'base_frame': launch_configuration['namespace']+'/base_link',
                'scan_topic': launch_configuration['namespace']+'/scan',
                'use_lifecycle_manager': use_lifecycle_manager,}
            ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        namespace=''
    )

    configure_event = EmitEvent(
        event=ChangeState(
        lifecycle_node_matcher=matches_action(start_async_slam_toolbox_node),
        transition_id=Transition.TRANSITION_CONFIGURE
        ),
        condition=IfCondition(AndSubstitution(autostart, NotSubstitution(use_lifecycle_manager)))
    )

    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=start_async_slam_toolbox_node,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                LogInfo(msg="[LifecycleLaunch] Slamtoolbox node is activating."),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(start_async_slam_toolbox_node),
                    transition_id=Transition.TRANSITION_ACTIVATE
                ))
            ]
        ),
        condition=IfCondition(AndSubstitution(autostart, NotSubstitution(use_lifecycle_manager)))
    )

    # Initialize rviz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['--display-config=' + slam_rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition = IfCondition(launch_rviz)
    )

    return[start_async_slam_toolbox_node, configure_event, activate_event, rviz_node]

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

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the slamtoolbox, Ignored when use_lifecycle_manager is true.')

    declare_use_lifecycle_manager = DeclareLaunchArgument(
        'use_lifecycle_manager', default_value='true',
        description='Enable bond connection during node activation')

    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(get_package_share_directory("robotino_slamtoolbox"),'config', 'slam_params.yaml'),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node')

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_argument)
    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_launch_rviz_argument)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_lifecycle_manager)
    ld.add_action(declare_slam_params_file_cmd)

    # Add the actions to launch all nodes
    ld.add_action(OpaqueFunction(function=launch_nodes_withconfig))

    return ld
