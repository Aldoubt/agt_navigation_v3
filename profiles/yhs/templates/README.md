# Mounted configuration preparation

Copy fastlio_config.template.yaml to ../calibration/fastlio_config.yaml. `r_il` is the measured 3x3 row-major rotation from LiDAR into FAST-LIO IMU/body; `t_il` is its measured 3-vector translation. Keep `world_frame: odom`, `body_frame: body`, `esti_il: false` for the V3 consumer contract. Algorithm/filter/noise defaults are retained from Mapping's pinned FAST-LIO configuration, not measurements of YHS or its IMU. Review them with real timestamps/units at R5. URDF separately describes the measured physical IMU/body/base/LiDAR mounts; do not set all transforms to identity to make checks pass.

Set localization.yaml fastlio_config: calibration/fastlio_config.yaml. Mapping uses the same file and exports the exact loaded calibration. Complete the four calibration records with status VERIFIED, verified_by and nonempty measured values/version/evidence.

For projection_config, copy `/opt/mapping_ws/src/agt-lio-pgo-mapping/exporters/agt_pcd2grid_exporter/config/projection.yaml` out of the built image to this mounted profile, review projection/obstacle thresholds for the site, and point mapping.yaml to it. This is an existing Mapping configuration, not a new projection implementation.

For Livox config, use the MID360 JSON supplied by the pinned driver at `/opt/mapping_ws/src/external/livox_ros_driver2/config/`, replace host/sensor IP fields with measured network values, and set sensors.yaml livox_config to the mounted relative file. Do not put manufacturer example IP addresses into production profiles. The sensor is Ethernet/UDP, not serial.
