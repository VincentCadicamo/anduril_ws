import os
import shutil

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    vslam_share = get_package_share_directory('anduril_vslam')
    config_path = os.path.join(vslam_share, 'config', 'qual1', 'estimator_config.yaml')
    rviz_path = os.path.join(vslam_share, 'rviz', 'openvins.rviz')

    # ── Launch arguments ──────────────────────────────────────────────────────

    manual_control_arg = DeclareLaunchArgument(
        'manual_control', default_value='false',
        description='Launch the keyboard manual controller in its own terminal'
    )

    auto_offboard_arm_arg = DeclareLaunchArgument(
        'auto_offboard_arm', default_value='true',
        description='Have mav_bridge enter offboard + arm on first setpoint'
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Launch RViz2 visualisation'
    )

    use_cv_arg = DeclareLaunchArgument(
        'use_cv', default_value='true',
        description='Launch the gate_pose_node CV pipeline'
    )

    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='/workspaces/anduril_ws/src/anduril_cv/models/yolov26-pose-best.onnx',
        description='Absolute path to the YOLO-Pose .pt model file'
    )

    use_ekf_arg = DeclareLaunchArgument(
        'use_ekf', default_value='true',
        description='Launch the drone_ekf_node IMU+gate fusion filter'
    )

    # ── Nodes ─────────────────────────────────────────────────────────────────

    camera_bridge = Node(
        package='anduril_sim_bridge',
        executable='camera_bridge',
        name='camera_bridge',
        output='screen',
    )

    # Delayed 5 s so OpenVINS has time to start its filter before the first
    # IMU sample arrives (early samples during static init matter most).
    mav_bridge = TimerAction(
        period=5.0,
        actions=[Node(
            package='anduril_sim_bridge',
            executable='mav_bridge',
            name='mav_bridge',
            output='screen',
            parameters=[{
                'auto_offboard_arm': LaunchConfiguration('auto_offboard_arm'),
            }],
        )],
    )

    # Namespace 'ov_msckf' so all topics land under /ov_msckf/ (e.g.
    # /ov_msckf/odomimu, /ov_msckf/pathimu). Matches the RViz config and
    # any downstream nodes that subscribe to VIO output.
    openvins = Node(
        package='ov_msckf',
        executable='run_subscribe_msckf',
        namespace='ov_msckf',
        output='screen',
        parameters=[{
            'config_path': config_path,
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
        condition=IfCondition(LaunchConfiguration('rviz')),
        additional_env={
            'QT_AUTO_SCREEN_SCALE_FACTOR': '0',
            'QT_SCALE_FACTOR': '1',
            'QT_QPA_PLATFORM': 'xcb',

            # OPTIMIZATION: Try letting WSLg use your actual GPU.
            # If you experience a white viewport, change '1' to '0' rather than completely
            # falling back to the ultra-heavy llvmpipe software driver.
            'LIBGL_ALWAYS_SOFTWARE': '0',
            'MESA_D3D12_DEFAULT_ADAPTER_NAME': 'NVIDIA', # Or 'AMD' / 'Intel' depending on your hardware
        },
    )

    # Fix WSLg runtime-dir permissions: Qt requires 0700 but WSLg mounts it 0777.
    fix_runtime_dir = ExecuteProcess(
        cmd=['bash', '-c', 'chmod 700 "$XDG_RUNTIME_DIR" 2>/dev/null || true'],
        output='screen',
    )

    # Manual controller runs in its own terminal so pynput has a dedicated
    # focused window for keyboard capture.
    terminal = (
        shutil.which('gnome-terminal')
        or shutil.which('xterm')
        or shutil.which('konsole')
        or shutil.which('x-terminal-emulator')
    )

    inner_cmd = (
        'source /opt/ros/humble/setup.bash; '
        'source "$COLCON_PREFIX_PATH/setup.bash" 2>/dev/null || '
        'source ./install/setup.bash 2>/dev/null; '
        'exec ros2 run anduril_manual_control manual_control'
    )

    if terminal and 'gnome-terminal' in terminal:
        manual_cmd = [terminal, '--', 'bash', '-c', inner_cmd]
    else:
        # Use -fa (FreeType/fontconfig) instead of the default XLFD bitmap font,
        # which is not installed in this container image.
        manual_cmd = [
            terminal or 'xterm',
            '-fa', 'monospace', '-fs', '12',
            '-e', 'bash', '-c', inner_cmd,
        ]

    manual_control = ExecuteProcess(
        cmd=manual_cmd,
        output='screen',
        condition=IfCondition(LaunchConfiguration('manual_control')),
    )

    gate_pose_node = Node(
        package='anduril_cv',
        executable='gate_pose_node',
        name='gate_pose_node',
        output='screen',
        parameters=[{
            'model_path':    LaunchConfiguration('model_path'),
            'image_topic':   '/cam0/image_raw',
            'detect_conf':   0.5,
            'keypoint_conf': 0.5,
            'gate_size':     2.7,
            'fx': 320.0,
            'fy': 320.0,
            'cx': 320.0,
            'cy': 180.0,
        }],
        condition=IfCondition(LaunchConfiguration('use_cv')),
    )

    drone_ekf_node = Node(
        package='anduril_ekf',
        executable='drone_ekf_node',
        name='drone_ekf_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'imu_topic':       '/imu0',
            'gate_pose_topic': '/gate/pose',
            'odom_out_topic':  '/drone/odom',
            'sigma_acc':       0.3,
            'sigma_gyr':       0.02,
            'sigma_ba_rw':     0.002,
            'sigma_bg_rw':     2e-4,
            'chi2_thresh':     11.345,
        }],
        condition=IfCondition(LaunchConfiguration('use_ekf')),
    )

    return LaunchDescription([
        manual_control_arg,
        auto_offboard_arm_arg,
        rviz_arg,
        use_cv_arg,
        model_path_arg,
        use_ekf_arg,
        fix_runtime_dir,
        camera_bridge,
        mav_bridge,
        openvins,
        gate_pose_node,
        drone_ekf_node,
        rviz,
        manual_control,
    ])
