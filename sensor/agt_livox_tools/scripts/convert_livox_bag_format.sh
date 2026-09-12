#!/usr/bin/env bash
set -euo pipefail

INPUT=""
OUTPUT=""
MODE=""
SOURCE_TOPIC="/livox/lidar"
TARGET_TOPIC=""
IMU_TOPIC="/livox/imu"
KEEP_IMU=1
ALLOW_MISSING_POINT_TIME=0
LIDAR_ID=0
EXTRA_TOPICS=()

usage() {
  cat <<'EOF'
Usage:
  convert_livox_bag_format.sh --input BAG --output BAG --mode MODE [options]

Modes:
  custom_to_pointcloud2  Livox CustomMsg -> sensor_msgs/PointCloud2
  pointcloud2_to_custom  sensor_msgs/PointCloud2 -> Livox CustomMsg

Defaults:
  source lidar topic: /livox/lidar
  IMU topic:          /livox/imu (copied unchanged)
  target topic:
    custom_to_pointcloud2 -> /livox/lidar_pc2
    pointcloud2_to_custom -> /livox/lidar_custom

Options:
  --source-topic TOPIC
  --target-topic TOPIC
  --imu-topic TOPIC
  --no-imu
  --extra-topic TOPIC       May be repeated; copied unchanged from source bag.
  --lidar-id N              CustomMsg lidar_id when reconstructing from PointCloud2.
  --allow-missing-point-time
                            Permit PointCloud2 -> CustomMsg with offset_time=0 when
                            the source cloud has no offset_time/timestamp. This is
                            geometry-only and MUST NOT be treated as equivalent raw
                            Livox input for Fast-LIO2/Batch-LIO.

Examples:
  ros2 run agt_livox_tools convert_livox_bag_format \
    --input /data/custom_bag --output /data/custom_as_pc2 \
    --mode custom_to_pointcloud2

  ros2 run agt_livox_tools convert_livox_bag_format \
    --input /data/pc2_bag --output /data/pc2_as_custom \
    --mode pointcloud2_to_custom
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --input) INPUT="${2:?missing value}"; shift 2 ;;
    --output) OUTPUT="${2:?missing value}"; shift 2 ;;
    --mode) MODE="${2:?missing value}"; shift 2 ;;
    --source-topic) SOURCE_TOPIC="${2:?missing value}"; shift 2 ;;
    --target-topic) TARGET_TOPIC="${2:?missing value}"; shift 2 ;;
    --imu-topic) IMU_TOPIC="${2:?missing value}"; shift 2 ;;
    --no-imu) KEEP_IMU=0; shift ;;
    --extra-topic) EXTRA_TOPICS+=("${2:?missing value}"); shift 2 ;;
    --lidar-id) LIDAR_ID="${2:?missing value}"; shift 2 ;;
    --allow-missing-point-time) ALLOW_MISSING_POINT_TIME=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "ERROR: unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
done

if [[ -z "${INPUT}" || -z "${OUTPUT}" || -z "${MODE}" ]]; then
  usage >&2
  exit 2
fi
if [[ ! -e "${INPUT}" ]]; then
  echo "ERROR: input bag does not exist: ${INPUT}" >&2
  exit 3
fi
if [[ -e "${OUTPUT}" ]]; then
  echo "ERROR: output path already exists: ${OUTPUT}" >&2
  exit 3
fi
if [[ "${MODE}" != "custom_to_pointcloud2" && "${MODE}" != "pointcloud2_to_custom" ]]; then
  echo "ERROR: unsupported mode: ${MODE}" >&2
  exit 2
fi

if [[ -z "${TARGET_TOPIC}" ]]; then
  if [[ "${MODE}" == "custom_to_pointcloud2" ]]; then
    TARGET_TOPIC="/livox/lidar_pc2"
  else
    TARGET_TOPIC="/livox/lidar_custom"
  fi
fi
if [[ "${TARGET_TOPIC}" == "${SOURCE_TOPIC}" ]]; then
  echo "ERROR: source and target topic names must differ during conversion." >&2
  echo "Use the default converted topic and remap it during playback." >&2
  exit 4
fi

for cmd in ros2; do
  command -v "${cmd}" >/dev/null 2>&1 || { echo "ERROR: ${cmd} not found" >&2; exit 5; }
done

STRICT_POINT_TIME=true
if [[ "${ALLOW_MISSING_POINT_TIME}" -eq 1 ]]; then
  STRICT_POINT_TIME=false
fi

TMP_DIR="$(mktemp -d -t agt_livox_bag_convert.XXXXXX)"
BRIDGE_PID=""
REC_PID=""
cleanup() {
  set +e
  if [[ -n "${REC_PID}" ]] && kill -0 "${REC_PID}" 2>/dev/null; then
    kill -INT "${REC_PID}" 2>/dev/null
    wait "${REC_PID}" 2>/dev/null
  fi
  if [[ -n "${BRIDGE_PID}" ]] && kill -0 "${BRIDGE_PID}" 2>/dev/null; then
    kill -INT "${BRIDGE_PID}" 2>/dev/null
    wait "${BRIDGE_PID}" 2>/dev/null
  fi
  rm -rf "${TMP_DIR}"
}
trap cleanup EXIT INT TERM

echo "==> Source bag"
ros2 bag info "${INPUT}" || true

echo "==> Starting format bridge"
ros2 run agt_livox_tools livox_format_bridge --ros-args \
  -p mode:="${MODE}" \
  -p input_topic:="${SOURCE_TOPIC}" \
  -p output_topic:="${TARGET_TOPIC}" \
  -p lidar_id:="${LIDAR_ID}" \
  -p require_per_point_time:="${STRICT_POINT_TIME}" \
  >"${TMP_DIR}/bridge.log" 2>&1 &
BRIDGE_PID=$!

RECORD_TOPICS=("${TARGET_TOPIC}")
PLAY_TOPICS=("${SOURCE_TOPIC}")
if [[ "${KEEP_IMU}" -eq 1 ]]; then
  RECORD_TOPICS+=("${IMU_TOPIC}")
  PLAY_TOPICS+=("${IMU_TOPIC}")
fi
for topic in "${EXTRA_TOPICS[@]}"; do
  RECORD_TOPICS+=("${topic}")
  PLAY_TOPICS+=("${topic}")
done

echo "==> Starting recorder: ${RECORD_TOPICS[*]}"
ros2 bag record -o "${OUTPUT}" "${RECORD_TOPICS[@]}" \
  >"${TMP_DIR}/record.log" 2>&1 &
REC_PID=$!

# Give DDS discovery time before the source starts. This avoids losing the first scan.
sleep 2
if ! kill -0 "${BRIDGE_PID}" 2>/dev/null; then
  echo "ERROR: bridge exited before playback" >&2
  cat "${TMP_DIR}/bridge.log" >&2 || true
  exit 6
fi
if ! kill -0 "${REC_PID}" 2>/dev/null; then
  echo "ERROR: recorder exited before playback" >&2
  cat "${TMP_DIR}/record.log" >&2 || true
  exit 6
fi

echo "==> Playing source topics: ${PLAY_TOPICS[*]}"
set +e
ros2 bag play "${INPUT}" --topics "${PLAY_TOPICS[@]}"
PLAY_RC=$?
set -e
if [[ "${PLAY_RC}" -ne 0 ]]; then
  echo "ERROR: ros2 bag play failed with code ${PLAY_RC}" >&2
  exit "${PLAY_RC}"
fi

# Allow the final converted message to propagate before closing sqlite/mcap output.
sleep 1
kill -INT "${REC_PID}" 2>/dev/null || true
wait "${REC_PID}" 2>/dev/null || true
REC_PID=""
kill -INT "${BRIDGE_PID}" 2>/dev/null || true
wait "${BRIDGE_PID}" 2>/dev/null || true
BRIDGE_PID=""

if [[ ! -e "${OUTPUT}" ]]; then
  echo "ERROR: conversion finished but output bag was not created" >&2
  cat "${TMP_DIR}/bridge.log" >&2 || true
  cat "${TMP_DIR}/record.log" >&2 || true
  exit 7
fi

echo "==> Converted bag"
ros2 bag info "${OUTPUT}" || true

cat <<EOF

CONVERSION COMPLETE
mode:         ${MODE}
source topic: ${SOURCE_TOPIC}
target topic: ${TARGET_TOPIC}
output bag:   ${OUTPUT}

Recommended replay remap when a consumer expects /livox/lidar:
  ros2 bag play '${OUTPUT}' --remap '${TARGET_TOPIC}:=/livox/lidar'
EOF

if [[ "${MODE}" == "pointcloud2_to_custom" && "${ALLOW_MISSING_POINT_TIME}" -eq 1 ]]; then
  cat <<'EOF'

WARNING: --allow-missing-point-time was enabled. If the original PointCloud2 did
not contain offset_time or per-point timestamp, the generated CustomMsg is NOT
an equivalent raw Livox stream and should not be used to validate LIO deskew.
EOF
fi
