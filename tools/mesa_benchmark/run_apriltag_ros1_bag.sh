#!/usr/bin/env bash
set -eo pipefail

if [ "$#" -lt 2 ]; then
  echo "Usage: $0 SCENARIO_DIR ROBOT_ID" >&2
  echo "Example: RATE=1 START=20 DURATION=60 $0 runs/scenario_S2B robot_03" >&2
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
OUT="$SCENARIO_ABS/processed/apriltag/${ROBOT_ID}"

RATE="${RATE:-1.0}"
START="${START:-0}"
DURATION="${DURATION:-0}"
TAG_SIZE="${TAG_SIZE:-0.10}"
TAG_ID_MAX="${TAG_ID_MAX:-99}"
TAG_DECIMATE="${TAG_DECIMATE:-2.0}"
TAG_THREADS="${TAG_THREADS:-4}"
POST_SLEEP="${POST_SLEEP:-3}"
STARTUP_SLEEP="${STARTUP_SLEEP:-5}"

if [ ! -f "$BAG" ]; then
  echo "Missing bag: $BAG" >&2
  exit 1
fi

source /opt/ros/melodic/setup.bash

mkdir -p "$OUT"
rm -f "$OUT"/detections.bag "$OUT"/detections.bag.active "$OUT"/detections.bag.orig.active

SETTINGS="$OUT/settings.yaml"
TAGS="$OUT/tags.yaml"
LAUNCH="$OUT/apriltag.launch"
LAUNCH_LOG="$OUT/apriltag_ros.log"
RECORD_LOG="$OUT/record.log"
PLAY_LOG="$OUT/play.log"

cat > "$SETTINGS" <<EOF
tag_family: tag36h11
tag_threads: $TAG_THREADS
tag_decimate: $TAG_DECIMATE
tag_blur: 0.0
tag_refine_edges: 1
tag_debug: 0
max_hamming_dist: 2
publish_tf: false
transport_hint: raw
EOF

{
  echo "standalone_tags:"
  echo "  ["
  for tag_id in $(seq 0 "$TAG_ID_MAX"); do
    sep=","
    if [ "$tag_id" = "$TAG_ID_MAX" ]; then
      sep=""
    fi
    printf "    {id: %s, size: %s, name: tag_%s}%s\n" "$tag_id" "$TAG_SIZE" "$tag_id" "$sep"
  done
  echo "  ]"
} > "$TAGS"

cat > "$LAUNCH" <<EOF
<launch>
  <arg name="settings" />
  <arg name="tags" />
  <node pkg="apriltag_ros" type="apriltag_ros_continuous_node" name="apriltag_detector" clear_params="true" output="screen">
    <rosparam command="load" file="\$(arg settings)" />
    <rosparam command="load" file="\$(arg tags)" />
    <remap from="image_rect" to="/camera/color/image_raw" />
    <remap from="camera_info" to="/camera/color/camera_info" />
    <param name="publish_tag_detections_image" type="bool" value="false" />
  </node>
</launch>
EOF

LAUNCH_PID=""
RECORD_PID=""
cleanup() {
  set +e
  if [ -n "$RECORD_PID" ]; then
    kill -INT "$RECORD_PID" >/dev/null 2>&1 || true
    wait "$RECORD_PID" >/dev/null 2>&1 || true
  fi
  if [ -n "$LAUNCH_PID" ]; then
    rosnode kill /apriltag_detector >/dev/null 2>&1 || true
    kill -INT "$LAUNCH_PID" >/dev/null 2>&1 || true
    wait "$LAUNCH_PID" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT

echo "Running AprilTag extraction for $ROBOT_ID"
echo "  bag: $BAG"
echo "  out: $OUT/detections.bag"

roslaunch "$LAUNCH" settings:="$SETTINGS" tags:="$TAGS" > "$LAUNCH_LOG" 2>&1 &
LAUNCH_PID=$!
sleep "$STARTUP_SLEEP"

rosbag record --buffsize 256 --chunksize 768 \
  -O "$OUT/detections.bag" \
  /tag_detections /tf /tf_static > "$RECORD_LOG" 2>&1 &
RECORD_PID=$!
sleep 1

PLAY_ARGS=(-r "$RATE" -s "$START" -q "$BAG" --topics
  /tf
  /tf_static
  /camera/color/image_raw
  /camera/color/camera_info
)
if [ "$DURATION" != "0" ]; then
  PLAY_ARGS=(-r "$RATE" -s "$START" -u "$DURATION" -q "$BAG" --topics
    /tf
    /tf_static
    /camera/color/image_raw
    /camera/color/camera_info
  )
fi

rosbag play "${PLAY_ARGS[@]}" </dev/null > "$PLAY_LOG" 2>&1
sleep "$POST_SLEEP"

kill -INT "$RECORD_PID" >/dev/null 2>&1 || true
wait "$RECORD_PID" >/dev/null 2>&1 || true
RECORD_PID=""

rosnode kill /apriltag_detector >/dev/null 2>&1 || true
kill -INT "$LAUNCH_PID" >/dev/null 2>&1 || true
wait "$LAUNCH_PID" >/dev/null 2>&1 || true
LAUNCH_PID=""

if [ -f "$OUT/detections.bag.active" ]; then
  rosbag reindex "$OUT/detections.bag.active" >/dev/null 2>&1 || true
  mv "$OUT/detections.bag.active" "$OUT/detections.bag"
fi

if [ ! -s "$OUT/detections.bag" ]; then
  echo "Detection bag was not created. See $LAUNCH_LOG and $RECORD_LOG" >&2
  exit 1
fi

python - "$OUT/detections.bag" <<'PY'
import sys
import rosbag
bag_path = sys.argv[1]
msgs = 0
dets = 0
ids = {}
with rosbag.Bag(bag_path) as bag:
    for _, msg, _ in bag.read_messages(topics=["/tag_detections"]):
        msgs += 1
        for det in msg.detections:
            dets += 1
            for tag_id in det.id:
                ids[int(tag_id)] = ids.get(int(tag_id), 0) + 1
print("Detection messages: %d" % msgs)
print("Detections: %d" % dets)
print("Tag IDs: %s" % ", ".join("%s:%s" % kv for kv in sorted(ids.items())))
PY
