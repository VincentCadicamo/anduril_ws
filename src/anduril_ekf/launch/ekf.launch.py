from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    drone_ekf_node = Node(
        package="anduril_ekf",
        executable="drone_ekf_node",
        name="drone_ekf_node",
        output="screen",
        emulate_tty=True,
        parameters=[{
            "sigma_acc":       0.3,
            "sigma_gyr":       0.02,
            "sigma_ba_rw":     0.002,
            "sigma_bg_rw":     2e-4,
            "chi2_thresh":     11.345,
            "imu_topic":       "/imu0",
            "gate_pose_topic": "/gate/pose",
            "odom_out_topic":  "/drone/odom",
        }],
    )

    return LaunchDescription([drone_ekf_node])
