# Source stamp and epochs originate inside the controller at computation time.
std_msgs/Header header
string owner_id
string task_id
uint64 control_epoch
uint64 odom_epoch
builtin_interfaces/Time plan_stamp
geometry_msgs/Twist velocity
