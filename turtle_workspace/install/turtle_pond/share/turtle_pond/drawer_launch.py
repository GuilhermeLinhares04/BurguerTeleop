from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='py_turtlesim_drawer',
            executable='drawer',
            name='turtle_drawer'
        )
    ])
