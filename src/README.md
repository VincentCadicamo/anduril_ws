QUICK START
-----------
In the container one command launches everything

ros2 launch anduril_bringup full_stack.launch.py manual_control:=true

This starts:
- camera_bridge     (UDP 5600 JPEG stream --> /cam0/image_raw)
- mav_bridge        (MAVLink UDP 14550 --> /imu0, with test climb for VSLAM init)
- OpenVINS          (visual-inertial odometry --> /ov_msckf/odomimu)
- RViz              (trajectory visualization)

Without the test climb if navigation is hooked up because the drone needs to move for the imu to initialize:

ros2 launch anduril_bringup full_stack.launch.py


RUNNING INDIVIDUAL PIECES
-------------------------

Bridges only:
ros2 launch anduril_sim_bridge sim_bridge.launch.py

VSLAM only (does need the bridges running):
ros2 launch anduril_vslam vslam.launch.py

Bridge with test climb (run thenodes separately):
ros2 run anduril_sim_bridge camera_bridge
ros2 run anduril_sim_bridge mav_bridge --ros-args -p test_climb:=true


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

Check timestamps match:
ros2 topic echo /imu0 --field header.stamp --once --no-daemon
ros2 topic echo /cam0/image_raw --field header.stamp --once --no-daemon


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