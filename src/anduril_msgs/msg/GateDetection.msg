#This is for a single gate detected in the image.

std_msgs/Header header

# Corner order: top-left, top-right, bottom-right, bottom-left.

Pixel[4] corners

float32 confidence    # detector score, [0, 1]
int32 gate_id         # gate number 0,1,2,3 ... -1 if unknown