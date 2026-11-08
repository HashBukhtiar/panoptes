std_msgs/Header header     # holds stamp (timestamp) and frame_id (whose reference this data is from)
uint8 mode                 # holds which mode command came from

uint8 MODE_TELEOP = 0
uint8 MODE_SOURCE = 1

bool is_absolute           # whether it is relative or absolute measurements
float64 yaw                # left/right in rad, +ve for right
float64 pitch              # up/down in rad, +ve for left