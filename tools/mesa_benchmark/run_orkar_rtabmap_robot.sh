#!/usr/bin/env bash
set -eo pipefail

if [ "$#" -lt 2 ]; then
  echo "Usage: $0 SCENARIO_DIR ROBOT_ID" >&2
  echo "Example: RATE=3 DURATION=120 $0 runs/scenario_S2B robot_03" >&2
  exit 2
fi

SCENARIO_DIR="$1"
ROBOT_ID="$2"

ROOT_DIR="$(cd "$(dirname "$0")/../.." && pwd)"
if [ -d "$ROOT_DIR/$SCENARIO_DIR" ]; then
  SCENARIO_ABS="$(cd "$ROOT_DIR/$SCENARIO_DIR" && pwd)"
else
  SCENARIO_ABS="$(cd "$SCENARIO_DIR" && pwd)"
fi

BAG="$SCENARIO_ABS/raw/robots/${ROBOT_ID}.bag"
OUT_ROOT="${OUT_ROOT:-$SCENARIO_ABS/processed/rtabmap_orkar}"
GRAPH_OUT="${GRAPH_OUT:-$SCENARIO_ABS/processed/graphs_orkar}"
OUT="$OUT_ROOT/${ROBOT_ID}"
DB="$OUT/rtabmap.db"
LOG="$OUT/rtabmap_ros.log"
EXPORT_LOG="$OUT/export.log"

# ORKAR's S1 wrapper defaults to 0.5 for reliability. Override RATE for faster
# replay when the machine can keep up; keep the same RTAB-Map parameters.
RATE="${RATE:-0.5}"
START="${START:-0}"
DURATION="${DURATION:-0}"
FLUSH_SLEEP="${FLUSH_SLEEP:-12}"
RTABMAP_STARTUP_SLEEP="${RTABMAP_STARTUP_SLEEP:-8}"
APPROX_SYNC_MAX_INTERVAL="${APPROX_SYNC_MAX_INTERVAL:-0.05}"
SUBSCRIBE_SCAN="${SUBSCRIBE_SCAN:-false}"

if [ ! -f "$BAG" ]; then
  echo "Missing bag: $BAG" >&2
  exit 1
fi

source /opt/ros/melodic/setup.bash

mkdir -p "$OUT" "$GRAPH_OUT"
rm -f "$DB" "$LOG" "$EXPORT_LOG"

LAUNCH_PID=""
wait_or_term() {
  local pid="$1"
  local grace="${2:-15}"
  [ -z "$pid" ] && return 0
  kill -0 "$pid" >/dev/null 2>&1 || return 0
  kill -INT "$pid" >/dev/null 2>&1 || true
  for _ in $(seq 1 "$grace"); do
    kill -0 "$pid" >/dev/null 2>&1 || {
      wait "$pid" >/dev/null 2>&1 || true
      return 0
    }
    sleep 1
  done
  kill -TERM "$pid" >/dev/null 2>&1 || true
  sleep 2
  kill -KILL "$pid" >/dev/null 2>&1 || true
  wait "$pid" >/dev/null 2>&1 || true
}

cleanup() {
  set +e
  rosnode kill /rtabmap/rtabmap >/dev/null 2>&1 || true
  if [ -n "$LAUNCH_PID" ]; then
    wait_or_term "$LAUNCH_PID" 15
  fi
}
trap cleanup EXIT

echo "Running ORKAR RTAB-Map frontend for $ROBOT_ID"
echo "  bag: $BAG"
echo "  db:  $DB"

roslaunch rtabmap_ros rtabmap.launch \
  use_sim_time:=true \
  rtabmapviz:=false \
  rviz:=false \
  visual_odometry:=false \
  icp_odometry:=false \
  frame_id:=base_footprint \
  odom_topic:=/odom \
  odom_frame_id:= \
  odom_frame_id_init:=odom \
  map_frame_id:=map \
  publish_tf_map:=true \
  rgb_topic:=/camera/color/image_raw \
  depth_topic:=/camera/aligned_depth_to_color/image_raw \
  camera_info_topic:=/camera/color/camera_info \
  depth_camera_info_topic:=/camera/aligned_depth_to_color/camera_info \
  rgbd_sync:=true \
  approx_rgbd_sync:=true \
  approx_sync:=true \
  approx_sync_max_interval:="$APPROX_SYNC_MAX_INTERVAL" \
  queue_size:=30 \
  wait_for_transform:=0.5 \
  subscribe_scan:="$SUBSCRIBE_SCAN" \
  scan_topic:=/scan \
  database_path:="$DB" \
  args:="--delete_db_on_start --Rtabmap/DetectionRate 2 --RGBD/LinearUpdate 0 --RGBD/AngularUpdate 0 --Mem/STMSize 30 --RGBD/OptimizeMaxError 2.0 --RGBD/OptimizeFromGraphEnd true" \
  > "$LOG" 2>&1 &
LAUNCH_PID=$!

sleep "$RTABMAP_STARTUP_SLEEP"

PLAY_TOPICS=(
  /tf
  /tf_static
  /odom
  /camera/color/image_raw
  /camera/color/camera_info
  /camera/aligned_depth_to_color/image_raw
  /camera/aligned_depth_to_color/camera_info
)
if [ "$SUBSCRIBE_SCAN" = "true" ]; then
  PLAY_TOPICS+=(/scan)
fi

PLAY_ARGS=(--clock -r "$RATE" -s "$START" -q "$BAG" --topics "${PLAY_TOPICS[@]}")
if [ "$DURATION" != "0" ]; then
  PLAY_ARGS=(--clock -r "$RATE" -s "$START" -u "$DURATION" -q "$BAG" --topics "${PLAY_TOPICS[@]}")
fi

rosbag play "${PLAY_ARGS[@]}" </dev/null
sleep "$FLUSH_SLEEP"
rosnode kill /rtabmap/rtabmap >/dev/null 2>&1 || true
wait_or_term "$LAUNCH_PID" 15
LAUNCH_PID=""
sleep 2

if [ ! -s "$DB" ]; then
  echo "RTAB-Map DB was not created. See $LOG" >&2
  exit 1
fi

rtabmap-export --poses --poses_format 4 \
  --output "${ROBOT_ID}_graph" \
  --output_dir "$GRAPH_OUT" \
  "$DB" > "$EXPORT_LOG" 2>&1

rm -f "$GRAPH_OUT/${ROBOT_ID}_graph_cloud.ply"

GRAPH="$GRAPH_OUT/${ROBOT_ID}_graph_poses.g2o"
if [ ! -s "$GRAPH" ]; then
  echo "g2o graph was not created. See $EXPORT_LOG" >&2
  exit 1
fi

echo "Wrote $GRAPH"
