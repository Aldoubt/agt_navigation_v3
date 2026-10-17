# Measurements in the horizontal carrier frame at header.stamp; x forward, y left.
# Errors describe carrier relative to row: positive e_y is left of its centerline.
std_msgs/Header header
uint64 odom_epoch
bool ground_valid
bool row_valid
bool clearance_valid
float64 lateral_error_m
float64 heading_error_rad
float64 width_m
float64 boundary_left_m
float64 boundary_right_m
float64 clearance_left_m
float64 clearance_right_m
float64 clearance_front_m
float64 observed_forward_m
float64[4] error_covariance
float64 quality
float64 quality_uncertainty
float64 quality_support
float64 quality_temporal
float64 quality_width
uint32 left_support
uint32 right_support
uint32 frame_support
string[] reasons
