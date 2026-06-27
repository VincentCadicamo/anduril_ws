import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Run the OpenVINS estimator only.

    Subscribes to a live camera topic and an IMU topic — no RViz, no bag
    playback. The estimator subscribes to the topics named in the Kalibr
    config (/cam0/image_raw and /imu0 by default); the cam_topic / imu_topic
    launch arguments remap those to whatever your source publishes on.
    """
    pkg_share = get_package_share_directory("anduril_vslam")
    config_path = os.path.join(pkg_share, "config", "qual1", "estimator_config.yaml")

    cam_topic_arg = DeclareLaunchArgument(
        name="cam_topic",
        default_value="/cam0/image_raw",
        description="Camera image topic the estimator subscribes to.",
    )
    imu_topic_arg = DeclareLaunchArgument(
        name="imu_topic",
        default_value="/imu0",
        description="IMU topic the estimator subscribes to.",
    )

    ov_node = Node(
        package="ov_msckf",
        executable="run_subscribe_msckf",
        namespace="ov_msckf",
        output="screen",
        parameters=[
            {"verbosity": "INFO"},
            {"use_stereo": False},
            {"max_cameras": 1},
            {"save_total_state": False},
            {"config_path": config_path},
        ],
        remappings=[
            ("/cam0/image_raw", LaunchConfiguration("cam_topic")),
            ("/imu0", LaunchConfiguration("imu_topic")),
        ],
    )

    return LaunchDescription([
        cam_topic_arg,
        imu_topic_arg,
        ov_node,
    ])
