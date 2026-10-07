# CURRENT_LOCALIZATION_ASSET_CONTRACT

Audited V3 e27abeb3fd63280ee8e48a47966e06588f8eebd8.

## Formal mapping input

map.pcd in optimized map coordinates; patches/<id>.pcd in FAST-LIO body coordinates. poses.txt records `patch_name x y z qw qx qy qz`; poses_timed.txt inserts timestamp seconds after patch_name. Pose is T_map_body from optimized PGO keyposes. Mapping keyframe admission uses translation/rotation thresholds. K in relocalization is **candidate_top_k**, configurable, native default 2. It does not define mapping keyframe sampling.

## Build (reuse, do not reimplement)

navigation/localization/agt_global_relocalization_native/src/build_relocalization_candidates.cpp reads poses.txt and patches. It skips absent/small patches (min_patch_points default 300), stores patch_name, optimized translation/orientation and gravity-level yaw-neutral polar descriptor. Output: polar_context.db and polar_context.yaml (entries/skipped and descriptor config). It does NOT read poses_timed.txt. Appliance validates every input patch/pose before invoking it and records per-keyframe hashes, pose and source identity.

build_relocalization_assets.cpp reads final map.pcd to build an actual **BBS voxel hierarchy**, NOT fake keyframes. Outputs: global_map_downsampled.pcd, relocalization_assets.yaml, voxelmaps_coords/voxel_params.txt and hierarchy *.pcd. These names come from V3; there is no invented bbs_index format.

## Runtime reads

agt_global_relocalization/.../global_relocalization.py uses candidate_sdk_command when polar_context.db exists; fallback whole-map BBS also exists. Config candidate command invokes candidate_bbs_gicp_localizer. Native candidate implementation reads polar DB, map/global downsampled and voxelmaps_coords. Descriptor candidates bound local BBS search; small_gicp target is local crop of optimized map around coarse hit, with full-map fallback if undersized; source is current accumulated live query. Default query_frame_mode=mapping_body. T_map_body result is converted once to T_map_base through configured body/base calibration. No RTK seed. Manual initial pose is explicit debug/fallback.

Continuous tracking: agt_map_tracker/.../map_tracker.py invokes native map_gicp_tracker, aligns live transformed query to local crop around T_map_odom*T_odom_base. Localization manager remains sole map->odom publisher.

## Existing package / activation

map_data_manager/agt_map_manager/.../map_package.py validates metadata schema_version 1, map_id/map_version/frame_id=map and assets localization_map, navigation_map, relocalization_assets. Required relocalization files are exactly the above DB/YAML/downsampled/voxel set. sha256_file and sha256_tree optional expected hashes are checked; appliance supplies mandatory hashes. Existing directory hash includes length-prefixed relative names then file hashes. Nav YAML image must stay inside package. Formal relocalization_contract requires T_map_body, body, mapping_body.

runtime_binding.py checks active_state schema/generation and exact metadata paths/identity. map_registry.py and map_manager.py own selection. Appliance writes compatible metadata at bundle root and one validated active pointer; consumers receive paths from the same binding. An active bundle is immutable; edits create a new version. Bundle additionally seals navigation PGM, review, mapping provenance and per-file hashes, which legacy V3 metadata alone does not fully enforce.

No native builder currently validates software commit/source map hash itself. Appliance wrapper performs full input verification, exact pose/patch correspondence, mandatory output hashes and provenance, then validates via V3 before use. Mock DB/index is marked mock and forbidden in real runtime; mock acceptance does not attest actual BBS/GICP quality.

## YHS composition fix

The existing global relocalizer's query frame contract has old Bunker mapping-body/Livox defaults independent of `body_to_base_calibration_file`. The field launch explicitly loads `mapping/calibration.yaml`, verifies its FAST-LIO internal transform agrees numerically with the mounted navigation FAST-LIO configuration, and supplies `mapping_body_livox_translation` and `mapping_body_livox_quaternion_xyzw`. This retains the original relocalizer/tracker and sole Localization Manager map→odom publisher. A different calibration requires a new mapping/bundle version, never an in-place edit.
