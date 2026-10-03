std_msgs/Header header
string task_id
agt_navigation_interfaces/RouteSegment segment
uint64 request_epoch
uint64 odom_epoch
bool active
bool anchor_valid
# Anchor frozen after trusted global acceptance; local terminal never rolls forward.
geometry_msgs/PoseStamped anchor_odom_pose
nav_msgs/Path authorized_odom_path
float64 local_distance_m
float64 local_elapsed_sec
bool handover_complete
bool recovery_complete
builtin_interfaces/Time plan_stamp
