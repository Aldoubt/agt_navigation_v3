# YHS + ordinary Mid-360 | 2026-10-08 field candidate

**NOT field-verified. Does not authorize chassis movement.**

This directory puts a ROS2-loadable URDF and official built-in IMU extrinsics directly into the YHS integration branch, so no USB transfer is required.

## Files
- `yhs_mid360_tf_nominal.urdf`: standalone **mesh-free** ROS2 URDF, preserving the 2026-10-08 SW export base_link-to-lidar offset (0.346672506, 0, 0.589682277 m) and +15-degree pitch. It adds `lidar_link -> imu_link` using ordinary Mid-360 IMU factory geometry (0.011, 0.02329, -0.04412 m), plus fixed CAD arm/track names. Its transparent box is an approximate visualization, **not** a collision geometry or Nav2 footprint.
- `fastlio_config_mid360_candidate.yaml`: official **inverse** `T_imu_lidar`, with `r_il=I`, `t_il=[-0.011,-0.02329,0.04412]`. The 15-degree chassis mounting angle must NOT be applied again in FAST-LIO.
- `imu_extrinsics.reference.yaml`: source provenance only, not a validated hardware record.

The original detailed five SolidWorks STL meshes (~26 MB uncompressed) are **not** in this folder. The previously supplied CAD ZIP retains those visuals. For localization/navigation testing the mesh-free URDF can publish static TF with no STL files; to show detailed visuals in RViz, the complete CAD mesh set still needs a separate transfer/upload. The original SW ROS1 display/gazebo launch files should not be run in Humble.

## Install into the field profile

First clone/update `feature/yhs-field-appliance-v1` and run `./install.sh`. Then from repo root:

```bash
mkdir -p "$HOME/agt/profiles/yhs/robot_description/urdf" "$HOME/agt/profiles/yhs/calibration"
cp appliance/assets/yhs_mid360/yhs_mid360_tf_nominal.urdf "$HOME/agt/profiles/yhs/robot_description/urdf/"
cp appliance/assets/yhs_mid360/fastlio_config_mid360_candidate.yaml "$HOME/agt/profiles/yhs/calibration/fastlio_config.yaml"
```

In `~/agt/profiles/yhs/robot.yaml` set `urdf: robot_description/urdf/yhs_mid360_tf_nominal.urdf`. Only set `lidar_frame: lidar_link`, `imu_frame: imu_link` after checking Livox message `header.frame_id`/TF. Set `fastlio_config: calibration/fastlio_config.yaml` in `localization.yaml`. Review all profile fields; **do not copy this YAML over a measured field calibration**.

Mapping and navigation must use the **same** FAST-LIO extrinsic source; changing extrinsics requires a new map version rather than silently using old 3D-BBS assets. The actual YHS model, `base_link` rotation center, `rotation_frame`, `base_footprint` height, LiDAR CAD origin versus Livox point-cloud O, CAN protocol/ROS1 driver, watchdog, footprint, speed and safety still require field verification.

### Afternoon field order

1. R0: build Docker/Qt and run mock, then diagnose actual profile. 
2. R1-R5: audit vendor ROS1 Noetic driver, CAN, odom/chassis/estop, physical remote/stop, LiDAR/IMU headers and TF without allowing motion.
3. R7-R8: physically remote-drive a short site map; clean finish/PGO; generate native keyframe patches, 3D-BBS/Polar Context, 2D raster; review/seal/activate using MapStudio.
4. R9: cold-start relocalization at several distinct poses; check map->odom authority, map hash, GICP pose and quality. Note: in the **existing** field appliance, START_NAVIGATION requires the full motion hardware profile even for a static localization test. Do not falsify `VERIFIED` or disable guard to bypass this gate.
5. R10-R13 only after watchdog, footprint, base transform, localization and manual emergency stop pass.

See `appliance/docs/REAL_ROBOT_TODO.md` and `appliance/docs/real_robot_acceptance.md`.

References: Livox Mid-360 User Manual v1.2, April 2024, p14 (https://www.livoxtech.com/cn/mid-360/downloads); 2026-10-08 user SolidWorks URDF export.
