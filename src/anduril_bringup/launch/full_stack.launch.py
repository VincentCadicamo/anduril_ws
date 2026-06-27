import os
import shutil
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction


def generate_launch_description():
    vslam_share = get_package_share_directory('anduril_vslam')
    config_path = os.path.join(vslam_share, 'config', 'qual1', 'estimator_config.yaml')
    rviz_path = os.path.join(vslam_share, 'rviz', 'openvins.rviz')

    manual_control_arg = DeclareLaunchArgument(
        'manual_control', default_value='false',
        description='Launch the keyboard manual controller in its own terminal'
    )

    auto_offboard_arm_arg = DeclareLaunchArgument(
        'auto_offboard_arm', default_value='true',
        description='Have mav_bridge enter offboard + arm on first setpoint'
    )

    camera_bridge = Node(
        package='anduril_sim_bridge',
        executable='camera_bridge',
        name='camera_bridge',
        output='screen',
    )

    mav_bridge = Node(
        package='anduril_sim_bridge',
        executable='mav_bridge',
        name='mav_bridge',
        output='screen',
        parameters=[{
            'auto_offboard_arm': LaunchConfiguration('auto_offboard_arm'),
        }],
    )

    openvins = Node(
        package='ov_msckf',
        executable='run_subscribe_msckf',
        name='ov_msckf',
        output='screen',
        parameters=[{
            'config_path': config_path,
            'verbosity': 'INFO',
            'use_stereo': False,
            'max_cameras': 1,
        }],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_path],
        output='screen',
    )

    # Run the manual controller in its own terminal window so pynput has a
    # dedicated focused window for keyboard capture. Pick whichever terminal
    # emulator is installed (xterm is the most common in dev containers).
    terminal = (
            shutil.which('gnome-terminal')
            or shutil.which('xterm')
            or shutil.which('konsole')
            or shutil.which('x-terminal-emulator')
    )

    # The command that actually runs inside the new terminal. We re-source the
    # overlay because a fresh terminal won't inherit the launching shell's env,
    # then exec the node so signals propagate.
    inner_cmd = (
        'source /opt/ros/humble/setup.bash; '
        'source "$COLCON_PREFIX_PATH/setup.bash" 2>/dev/null || '
        'source ./install/setup.bash 2>/dev/null; '
        'exec ros2 run anduril_manual_control manual_control'
    )

    if terminal and 'gnome-terminal' in terminal:
        manual_cmd = [terminal, '--', 'bash', '-c', inner_cmd]
    else:
        # xterm / konsole / x-terminal-emulator all accept -e
        manual_cmd = [terminal or 'xterm', '-e', 'bash', '-c', inner_cmd]

    manual_control = ExecuteProcess(
        cmd=manual_cmd,
        output='screen',
        condition=IfCondition(LaunchConfiguration('manual_control')),
    )

    delayed_mav_bridge = TimerAction(
        period=5.0,  # Delays arming/takeoff for 5 seconds to let OpenVINS initialize
        actions=[mav_bridge]
    )

    return LaunchDescription([
        manual_control_arg,
        auto_offboard_arm_arg,
        camera_bridge,
        delayed_mav_bridge,
        openvins,
        rviz,
        manual_control,
    ])