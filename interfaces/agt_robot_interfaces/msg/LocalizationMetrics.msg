# Machine-readable localization diagnostics. Yaw fields are radians.
string state
string tracking_health
string reason

# Current T_map_odom translation and yaw.
float64 map_odom_x
float64 map_odom_y
float64 map_odom_z
float64 map_odom_yaw

# Change from the previous metrics sample.
float64 correction_translation_delta
float64 correction_yaw_delta

# Most recent map-tracking innovation. Yaw is radians.
float64 tracking_innovation_translation
float64 tracking_innovation_yaw

# Global localization covariance summary. Position is metres; yaw is radians.
float64 global_position_std
float64 global_yaw_std

builtin_interfaces/Time stamp
