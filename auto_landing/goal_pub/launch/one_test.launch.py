import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from launch_ros.actions import Node

def generate_launch_description():
    apriltag_params_file = PathJoinSubstitution([
        FindPackageShare('apriltag_ros'),
        'cfg',
        'tags_Standard41h12.yaml'
        # 'tags_36h11.yaml'
    ])

    return LaunchDescription(
        [
            ExecuteProcess(
                cmd=["ros2", "launch", "image_proc", "image_proc.launch.py"], output="screen"
            ),
            ExecuteProcess(
                cmd=["ros2", "run", "px4_offboard", "bezier_control"], output="screen"
            ),
            ExecuteProcess(
                cmd=["ros2", "run", "goal_pub", "tag_pub_2"], output="screen"
            ),
            ExecuteProcess(
                cmd=["ros2", "run", "px4_offboard", "land_test"], output="screen"
            ),
            ExecuteProcess(
                cmd=[
                    "ros2", "run", "apriltag_ros", "apriltag_node",
                    "--ros-args",
                    "-r", "image_rect:=/image_rect_color",
                    "-r", "camera_info:=/camera_info",
                    "--params-file", apriltag_params_file
                ],
                output="screen"
            ),
            ExecuteProcess(
                cmd=["ros2", "run", "goal_pub", "topic_name"], output="screen"
            ),
        ]
    )
