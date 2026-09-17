# Runtime observability for the canonical Batch-LIO odometry adapter.

uint8 STATE_FRESH=0
uint8 STATE_INPUT_DELAYED=1
uint8 STATE_STALE=2
uint8 STATE_NO_INPUT=3

builtin_interfaces/Time stamp
uint8 state

# Age of the most recently received Batch-LIO input header at publication time.
# A negative value means the adapter has not received a valid header timestamp.
float64 input_age_sec

# Wall/ROS-clock age since the adapter last accepted and published a pose.
# A negative value means the adapter has not yet accepted any pose.
float64 last_valid_pose_age_sec

uint64 accepted_count
uint64 rejected_count

# Rolling accepted odometry/TF publication rate over the diagnostic window.
float64 publish_rate
