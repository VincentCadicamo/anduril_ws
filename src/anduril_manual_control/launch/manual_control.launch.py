from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('anduril_manual_control'),
        'config',
        'manual_control.yaml',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'setpoint_topic',
            default_value='/setpoint/attitude_target',
            description='Topic to publish AttitudeTarget setpoints on.',
        ),
        Node(
            package='anduril_manual_control',
            executable='manual_control',
            name='manual_control',
            output='screen',
            emulate_tty=True,
            parameters=[
                config,
                {'setpoint_topic': LaunchConfiguration('setpoint_topic')},
            ],
        ),
    ])