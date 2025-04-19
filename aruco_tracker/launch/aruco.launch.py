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
                '--x', '0.05', '--y', '0', '--z', '0.5',
                '--qw','0.5','--qx','0.5','--qy','0.5','--qz','0.5', '--frame-id', 'robotinobase2/base_link', '--child-frame-id', 'robotinobase2/camera_link']
        ),
        Node(
             package='aruco_tracker',
             executable='aruco',
             name='aruco',
         ),
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            parameter=[{'video_device':'/dev/video0'}]
        )
    ])
