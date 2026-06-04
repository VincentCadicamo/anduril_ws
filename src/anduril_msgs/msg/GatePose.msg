#This is for the gate pose relative to the camera from the PnP solver

#header.frame_id should be the camera optical frame.

std_msgs/Header header

geometry_msgs/Pose pose    # gate origin in camera frame
float32 confidence
int32 gate_id