# Global Relocalization SDK Contract

`agt_global_relocalization` deliberately separates ROS integration from the concrete proprietary/community SDK.

## ROS side

Input:
- `/agt/relocalization/request` (`std_msgs/Empty`)
- `/agt/livox/points` (`sensor_msgs/PointCloud2`)

Output:
- `/agt/relocalization/pose` (`geometry_msgs/PoseWithCovarianceStamped`)
- `/agt/global_relocalization/status` (`std_msgs/String`, JSON diagnostics)

The adapter never publishes `map -> odom`. `agt_localization_manager` remains the exclusive owner of that TF.

## External SDK wrapper contract

Configure `sdk_command` in `config/global_relocalization.yaml`.

Available command placeholders:
- `{scan_pcd}`: temporary accumulated query scan PCD
- `{global_map}`: configured global map PCD/path
- `{timeout_sec}`: configured timeout

Example:

```yaml
sdk_command: "/opt/agt_sdk/bin/localize_once --map {global_map} --scan {scan_pcd} --timeout {timeout_sec}"
```

The wrapper process must exit with code 0 and print a JSON object as its last stdout line:

```json
{
  "success": true,
  "x": 12.3,
  "y": -4.5,
  "z": 0.2,
  "qx": 0.0,
  "qy": 0.0,
  "qz": 0.382683,
  "qw": 0.923880,
  "score": 0.91,
  "fitness": 0.18,
  "overlap": 0.63,
  "message": "matched"
}
```

`score` is normalized to `[0,1]` by the wrapper. `fitness` is lower-better. `overlap` is normalized to `[0,1]`.

Until the SDK exposes calibrated covariance, the adapter maps score conservatively into position/yaw covariance. Once a native SDK API is available, replace only the backend layer; keep these ROS contracts unchanged.

## Failure policy

No pose is published when:
- query scan is missing/too small;
- global map is missing;
- SDK times out or returns non-zero;
- JSON is malformed;
- quaternion is invalid;
- score / fitness / overlap gates fail.

A failed global attempt therefore cannot overwrite the current global correction.
