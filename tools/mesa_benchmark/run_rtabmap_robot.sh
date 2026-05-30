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
OUT="$SCENARIO_ABS/processed/rtabmap/${ROBOT_ID}"
GRAPH_OUT="$SCENARIO_ABS/processed/graphs"
DB="$OUT/rtabmap.db"
LOG="$OUT/rtabmap_ros.log"
EXPORT_LOG="$OUT/export.log"

RATE="${RATE:-3.0}"
START="${START:-0}"
DURATION="${DURATION:-0}"
FLUSH_SLEEP="${FLUSH_SLEEP:-8}"
RTABMAP_STARTUP_SLEEP="${RTABMAP_STARTUP_SLEEP:-8}"

if [ ! -f "$BAG" ]; then
  echo "Missing bag: $BAG" >&2
  exit 1
fi

source /opt/ros/melodic/setup.bash

mkdir -p "$OUT" "$GRAPH_OUT"
rm -f "$DB" "$LOG" "$EXPORT_LOG"

LAUNCH_PID=""
cleanup() {
  set +e
  rosnode kill /rtabmap/rtabmap >/dev/null 2>&1 || true
  if [ -n "$LAUNCH_PID" ]; then
    kill "$LAUNCH_PID" >/dev/null 2>&1 || true
    wait "$LAUNCH_PID" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT

echo "Running RTAB-Map frontend for $ROBOT_ID"
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
  rgb_topic:=/camera/color/image_raw \
  depth_topic:=/camera/aligned_depth_to_color/image_raw \
  camera_info_topic:=/camera/color/camera_info \
  subscribe_scan:=true \
  scan_topic:=/scan \
  approx_sync:=true \
  queue_size:=30 \
  wait_for_transform:=0.5 \
  database_path:="$DB" \
  args:="--delete_db_on_start --Reg/Force3DoF true --Reg/Strategy 2 --RGBD/NeighborLinkRefining true --RGBD/ProximityBySpace true --RGBD/ProximityPathMaxNeighbors 10 --RGBD/LinearUpdate 0.08 --RGBD/AngularUpdate 0.08" \
  > "$LOG" 2>&1 &
LAUNCH_PID=$!

sleep "$RTABMAP_STARTUP_SLEEP"

PLAY_ARGS=(--clock -r "$RATE" -s "$START" -q "$BAG" --topics
  /tf
  /tf_static
  /odom
  /scan
  /camera/color/image_raw
  /camera/color/camera_info
  /camera/aligned_depth_to_color/image_raw
  /camera/aligned_depth_to_color/camera_info
)
if [ "$DURATION" != "0" ]; then
  PLAY_ARGS=(--clock -r "$RATE" -s "$START" -u "$DURATION" -q "$BAG" --topics
    /tf
    /tf_static
    /odom
    /scan
    /camera/color/image_raw
    /camera/color/camera_info
    /camera/aligned_depth_to_color/image_raw
    /camera/aligned_depth_to_color/camera_info
  )
fi

rosbag play "${PLAY_ARGS[@]}" </dev/null
sleep "$FLUSH_SLEEP"
rosnode kill /rtabmap/rtabmap >/dev/null 2>&1 || true
wait "$LAUNCH_PID" >/dev/null 2>&1 || true
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
