QUICK START
-----------

Fly manually with keyboard control:

    ros2 launch anduril_bringup full_stack.launch.py manual_control:=true

Autonomous / navigation stack (no keyboard) (not completed):

    ros2 launch anduril_bringup full_stack.launch.py

Both commands start:
- camera_bridge     UDP 5600 JPEG stream  -->  /cam0/image_raw
- mav_bridge        MAVLink UDP 14550     -->  /imu0, /sim/odometry, /race/status, /race/track ...
- OpenVINS          visual-inertial VIO   -->  /ov_msckf/odomimu, /ov_msckf/pathimu ...
- RViz2             trajectory + FPV feed

mav_bridge is delayed 5 s on startup to let OpenVINS initialise its filter before
IMU data arrives.


LAUNCH ARGUMENTS
----------------

    ros2 launch anduril_bringup full_stack.launch.py [arg:=value ...]

  manual_control:=true/false   Open a terminal window with keyboard control (default: false)
  auto_offboard_arm:=true/false  Enter offboard mode and arm on first setpoint (default: true)
  rviz:=true/false             Launch RViz2 visualisation (default: true)

Examples:

  # Keyboard control, no RViz (headless dev container)
  ros2 launch anduril_bringup full_stack.launch.py manual_control:=true rviz:=false

  # Autonomous run, RViz on
  ros2 launch anduril_bringup full_stack.launch.py

  # Manual control keys (once the drone is armed with R):
  #   W/S  pitch fwd/back    A/D  roll left/right
  #   Q/E  yaw left/right    Space/Shift  up/down
  #   R    arm               P    quit


RUNNING INDIVIDUAL PIECES
--------------------------

Sim bridges only:
    ros2 launch anduril_sim_bridge sim_bridge.launch.py

VIO estimator only (bridges must already be running):
    ros2 launch anduril_vslam vslam.launch.py

Individual nodes:
    ros2 run anduril_sim_bridge camera_bridge
    ros2 run anduril_sim_bridge mav_bridge
    ros2 run anduril_manual_control manual_control


USEFUL COMMANDS
---------------

Check data is flowing:
    ros2 topic hz /imu0 --no-daemon
    ros2 topic hz /cam0/image_raw --no-daemon
    ros2 topic hz /ov_msckf/odomimu --no-daemon

View camera feed:
    ros2 run rqt_image_view rqt_image_view /cam0/image_raw

View feature tracking:
    ros2 run rqt_image_view rqt_image_view /ov_msckf/trackhist

Check timestamps match (IMU and camera must share sim timebase):
    ros2 topic echo /imu0 --field header.stamp --once --no-daemon
    ros2 topic echo /cam0/image_raw --field header.stamp --once --no-daemon

Monitor race state:
    ros2 topic echo /race/status --no-daemon
    ros2 topic echo /race/track --once --no-daemon


WINDOWS NETWORKING SETUP
-------------------------

The sim only sends to 127.0.0.1. For packets to reach WSL2,
create C:\Users\<you>\.wslconfig:

    [wsl2]
    networkingMode=mirrored
    memory=8GB

    [experimental]
    hostAddressLoopback=true

Then from PowerShell: wsl --shutdown
Reopen WSL. Localhost traffic now crosses into WSL.
