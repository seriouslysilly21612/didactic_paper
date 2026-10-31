std_msgs/Header header

# Original COLOR capture time, carried end-to-end for latency metrics. Kept
# SEPARATE from header.stamp (which stays the depth-frame stamp; header.frame_id
# = camera_depth_optical_frame is what the base node uses for the TF lookup).
# That collision is why PickTarget carries capture_stamp too - rationale there.
builtin_interfaces/Time capture_stamp

bool target_valid
bool depth_valid

int32 class_id
string class_name
float32 confidence

float32 center_x
float32 center_y
float32 width
float32 height

float32 x
float32 y
float32 z
