from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="anduril_sim_bridge",
            executable="camera_bridge",
            name="camera_bridge",
            output="screen",
        ),
        Node(
            package="anduril_sim_bridge",
            executable="mav_bridge",
            name="mav_bridge",
            output="screen",
        ),
    ])