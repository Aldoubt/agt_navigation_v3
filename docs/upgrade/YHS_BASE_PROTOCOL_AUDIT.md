# YHS TK-mid Base Protocol Audit

Status: **BLOCKED**. The local source tree and manuals do not establish a safe, unambiguous command contract for the physical chassis. No YHS base or motion command was started during this audit.

## Audited local sources

| Source | Snapshot | Evidence |
|---|---|---|
| ROS 2 driver checkout `/home/yangxuan/ros2_ws/src/drivers/yhs_tk_mid_ros2` | detached `6d002bd` | `yhs_can_control/src/yhs_can_control_node.cpp`, `yhs_can_interfaces/msg/{CtrlCmd,CtrlFb}.msg`, `README.md`, included ROS 2 manual V1.1.1 dated 2026-08-08 |
| ROS 2 audit copy `/home/yangxuan/ros2_ws/experiments/yhs_driver_audit/TK-mid-ros2` | local copy | Driver source SHA-256 matches the checkout; manual PDF SHA-256 matches the checked-in ROS 2 PDF |
| ROS 1 audit copy `/home/yangxuan/ros2_ws/experiments/yhs_driver_audit/TK-mid-ros` | local copy | `yhs_can_control/src/yhs_can_control.cpp`, `yhs_can_msgs/msg/{ctrl_cmd,ctrl_fb}.msg`, README and manual V1.1.2 dated 2026-08-08 |
| Additional ROS 1 copies | local copies | `机械臂/kiwirobot/TK-mid-ros1`, `机械臂/slam_sec_ws/src/car_base/{TK-mid-ros1,yhs_can_control}`, `机械臂/slam_ws/src/TK-mid-ros1`, and `机械臂/test_ws/src/yhs_can_control`; the four ROS 1 driver source files have the same SHA-256 |
| Platform bridge/config | `agt_robot_platform` `05b7782` | `agt_yhs_adapter`, `robot_hardware.launch.py`, `config/robots/yhs_harvesting/{robot,devices}.yaml` |

The ROS 2 checkout is the only YHS driver selected by the current ROS 2 bringup. Its whole-robot config remains blocked and has no confirmed drive gear. The ROS 1 copies are historical evidence only; they are not launch owners in the current ROS 2 stack.

## Facts visible in source

- The ROS 2 node consumes `ctrl_cmd`, `io_cmd`, and `free_ctrl_cmd`; it publishes `chassis_info_fb` and integrated `odom`. The platform launch remaps these beneath `/yhs` and maps `odom` to `/wheel/odom`. This ROS 2 driver does not broadcast TF.
- The normal command frame is extended CAN ID `0x98C4D1D0`; the free-wheel command frame is `0x98C4D2D0`. The driver packs `ctrl_cmd_linear * 1000` and `ctrl_cmd_angular * 100` into signed integer fields and appends an XOR checksum. This reports the implementation; it does not verify the physical controller interprets the fields the same way.
- The ROS 2 message fields are `ctrl_cmd_gear`, `ctrl_cmd_linear`, and `ctrl_cmd_angular`. The platform bridge converts `Twist.angular.z` radians/s to degrees/s before assigning `ctrl_cmd_angular`.
- The ROS 2 driver publishes chassis feedback including control and IO feedback. E-stop and joypad/remote indicators appear in the ROS 2 README/message documentation, but the command bridge does not gate outgoing commands on those feedback values.
- The ROS 1 driver copies can optionally broadcast `odom -> base_link` when `tfUsed` is enabled. It defaults false in that source, but the old workspaces are not valid owners for the current navigation TF chain.
- Source reads found no wheelbase, steering linkage model, steering limits/rate, or documented Ackermann/swerve kinematic implementation in the audited driver/config trees.

## Conflicting protocol documentation

- ROS 2 README documents gear values `0=disable, 1=P, 2=R, 3=N, 4=D`, a target steering angle in degrees, and a recommended command rate of at least 30 Hz.
- The included ROS 2 V1.1.1 manual instead documents `0=disable, 1=park, 2=neutral, 3=kinematic control`, a target body velocity and target body steering angle in degrees, with command publication at **50 Hz or higher**.
- The ROS 1 V1.1.2 manual likewise documents gear `3` as kinematic control, not the ROS 2 README's `3=N, 4=D` table.
- The manuals describe a steering-angle field; the checked-out ROS 2 interface calls the field `ctrl_cmd_angular`, and the driver turns the received value into an angular odometry rate (`degrees * pi / 180`). The local sources do not establish whether this vehicle firmware expects a steering angle, yaw rate, or another quantity for this particular model/firmware.
- The software bridge's degrees/s conversion is therefore not evidence that the physical command semantics are correct.

## Required physical facts still missing

The following must be tied to the actual vehicle serial/configuration and firmware before any YHS profile or adapter can be enabled:

1. Manufacturer-confirmed gear/mode values and whether gear 3 is required for the installed controller.
2. Exact command field meanings, units, sign conventions, legal range, saturation behavior, and command watchdog behavior.
3. Actual chassis kinematics and measured geometry: tracked/skid, Ackermann, swerve, or another model; wheelbase/track/steering linkage as applicable; turning constraints and reverse behavior.
4. Physical remote/manual arbitration and emergency-stop behavior when CAN commands are present, including the behavior after stale CAN data, disconnect, driver restart, and a pressed E-stop.
5. Whether feedback fields report the actual control mode and whether the current driver parses them correctly for the fitted firmware.
6. Agreement between the actual vehicle's protocol revision, the checked-in driver revision, and the applicable manufacturer manual.

## Decision

Keep YHS `BLOCKED`. Do not classify it as Ackermann, tracked/skid-steer, or swerve from the directory name or generic manual. Do not fill in `yhs_drive_gear`, wheelbase, steering limits, or a Nav2 field profile by inference. Keep the current bringup refusal when drive gear is unset, and do not launch the physical YHS driver as part of these software-only phases.

### P4 software enforcement update (2026-10-02)

The platform adapter registry now marks YHS `blocked` independently of the
whole-robot YAML status, and the atomic hardware launch helper raises before
creating a YHS driver or command bridge action. P4 `--show-args` and source
tests confirm the launch remains fail-closed; this is software evidence only
and does not change any unresolved physical fact above.

The next audit can only close this block with vehicle-specific written protocol/kinematic confirmation and controlled bench evidence approved by the operator. No motion test was run or authorized here.

## Evidence not collected

- `BLOCKED`: vehicle serial/firmware-specific protocol confirmation.
- `NOT_RUN`: CAN capture against the physical YHS controller.
- `NOT_RUN`: supervised command/timeout/remote/E-stop behavior on real hardware.
- `NOT_RUN`: physical steering geometry and motion/odometry validation.
