from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                '--x', '-0.52', '--y', '0', '--z', '0',
                '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'base_link', '--child-frame-id', 'camera_link']
        ),
        Node(
             package='aruco_tracker',
             executable='aruco',
             name='aruco',
         ),
        Node(
            package='ros2_usb_camera',
            executable='usb_cam_node_exe',
            name='usb_cam',
            parameters=[{'video_device': '/dev/video2'}]
        )
    ])
