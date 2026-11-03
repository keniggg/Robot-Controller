# Arm-only, explicitly mapped ros_control command. Raw encoder feedback is unchanged.
std_msgs/Header header
string binding_id
string owner_token
string goal_token
uint64 epoch_ns
uint64 sequence
float64[6] physical_positions_rad
float64[6] wire_positions_rad
