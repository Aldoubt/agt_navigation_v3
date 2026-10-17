# valid requires actual evidence, not merely receipt of an Odometry message.
std_msgs/Header header
uint64 odom_epoch
bool valid
float64 quality
float64 position_std_m
float64 yaw_std_rad
float64 input_age_sec
float64 output_age_sec
string reason
