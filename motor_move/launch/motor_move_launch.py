from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='motor_move',       # Name of the package where the node is located
            executable='motor_move',    # Name of the executable to run
            name='motor_move',          # Name of the node
            output='screen',            # Directs the output to the screen
            parameters=[]               # List any ROS parameters here
        )
    ])
