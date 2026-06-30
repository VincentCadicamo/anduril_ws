# anduril_vslam

OpenVINS visual-inertial odometry configuration for the AI Grand Prix qualifier drone.

## Launch

**Full stack (sim bridge + VIO + RViz):**
```bash
ros2 launch anduril_bringup full_stack.launch.py
```

Optional arguments:
```bash
ros2 launch anduril_bringup full_stack.launch.py \
  rviz:=false \
  manual_control:=true \
  auto_offboard_arm:=false
```

**VIO estimator only (no RViz, no bridge):**
```bash
ros2 launch anduril_vslam vslam.launch.py
```

Optional arguments:
```bash
ros2 launch anduril_vslam vslam.launch.py \
  cam_topic:=/cam0/image_raw \
  imu_topic:=/imu0
```
