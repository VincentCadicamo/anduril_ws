from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    model_path_arg = DeclareLaunchArgument(
        "model_path",
        description="Absolute path to the YOLO-Pose .pt model file",
    )

    gate_pose_node = Node(
        package="anduril_cv",
        executable="gate_pose_node",
        name="gate_pose_node",
        parameters=[{
            "model_path":    LaunchConfiguration("model_path"),
            "image_topic":   "/cam0/image_raw",
            "detect_conf":   0.5,
            "keypoint_conf": 0.5,
            "gate_size":     2.7,
            # Camera intrinsics — spec §3.8
            "fx": 320.0,
            "fy": 320.0,
            "cx": 320.0,
            "cy": 180.0,
        }],
        output="screen",
        emulate_tty=True,
    )

    return LaunchDescription([
        model_path_arg,
        gate_pose_node,
    ])
