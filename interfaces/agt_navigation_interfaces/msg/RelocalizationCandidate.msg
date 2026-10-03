# pose is a historical observation at reference_stamp, never a pose at completion.
string job_id
uint64 request_epoch
uint64 odom_epoch
string map_id
string map_version
string map_hash
builtin_interfaces/Time reference_stamp
builtin_interfaces/Time completed_stamp
geometry_msgs/PoseWithCovarianceStamped pose
agt_navigation_interfaces/GlobalQuality quality
