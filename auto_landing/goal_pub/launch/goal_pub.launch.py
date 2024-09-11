import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([  
        Node(
            package='goal_pub',
            namespace='',
            executable='tag_pub',
            name='goal_pub'
        ),
        Node(
            package='goal_pub',
            namespace='',
            executable='filter',
            name='goal_pub'
        )
    ])
