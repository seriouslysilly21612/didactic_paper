# Original COLOR capture time, propagated from DetectionArray.header.stamp and
# forwarded unchanged through PickTarget3D.capture_stamp to the base node.
#
# Why a dedicated field and not a std_msgs/Header:
#   1. A header carries ONE (stamp, frame_id) pair, and at the next hop that pair
#      is already taken by depth - PickTarget3D.header = depth header, because the
#      base node does its TF lookup with frame_id = camera_depth_optical_frame.
#      The color stamp needs its own field there regardless; using the same name
#      here lets the 3D node forward it instead of translating header -> field.
#   2. header.stamp is auto-consumed by tf2 / message_filters / `ros2 topic delay`.
#      A copied color header would look routine and invite a wrong TF lookup, and
#      frame_id would be a lie here: center_x/y and width/height are PIXELS, not a
#      point expressed in any frame.
builtin_interfaces/Time capture_stamp

bool target_valid
int32 class_id
string class_name
float32 confidence
float32 center_x
float32 center_y
float32 width
float32 height
