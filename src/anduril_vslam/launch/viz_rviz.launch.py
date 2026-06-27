import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """RViz visualization for a running OpenVINS estimator.

    Run with vslam.launch.py
    shows the estimated path, odometry, MSCKF/SLAM feature points,
    and the feature-track image.

    Fixed Frame is preset to 'global' (the frame OpenVINS publishes in)
    """
    pkg_share = get_package_share_directory("anduril_vslam")
    rviz_path = os.path.join(pkg_share, "rviz", "openvins.rviz")

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_path, "--ros-args", "--log-level", "warn"],
        output="screen",
    )

    return LaunchDescription([rviz_node])
