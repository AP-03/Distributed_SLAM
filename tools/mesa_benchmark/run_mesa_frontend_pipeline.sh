#!/usr/bin/env bash
set -euo pipefail

if [ "$#" -lt 2 ]; then
  echo "Usage: $0 SCENARIO_DIR ROBOT_COUNT [RUN_NAME]" >&2
  echo "Example: RTABMAP_RATE=3 APRILTAG_RATE=4 $0 runs/scenario_S2B 6 s2b_mesa_apriltag" >&2
  exit 2
fi

SCENARIO="$1"
ROBOT_COUNT="$2"
RUN_NAME="${3:-mesa_apriltag}"

ROOT_DIR="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT_DIR"

RTABMAP_RATE="${RTABMAP_RATE:-3}"
APRILTAG_RATE="${APRILTAG_RATE:-4}"
APRILTAG_DECIMATE="${APRILTAG_DECIMATE:-3.0}"
APRILTAG_THREADS="${APRILTAG_THREADS:-4}"
APRILTAG_SIZE="${APRILTAG_SIZE:-0.10}"
APRILTAG_ID_MAX="${APRILTAG_ID_MAX:-99}"
TAG_RULE_SOURCE="${TAG_RULE_SOURCE:-best_unique_by_tag}"
TAG_TARGET_SHIFT="${TAG_TARGET_SHIFT:-0}"

GRAPH_DIR="$SCENARIO/processed/graphs_${RUN_NAME}"
RTABMAP_DIR="$SCENARIO/processed/rtabmap_${RUN_NAME}"
TAG_DIR="$SCENARIO/processed/apriltag"
TAG_CSV="$TAG_DIR/$(basename "$SCENARIO")_apriltag_detections.csv"
DIAG_DIR="$SCENARIO/processed/apriltag_diagnostic_${RUN_NAME}"
CONSTRAINT_DIR="$SCENARIO/processed/apriltag_constraints_${RUN_NAME}"
JRL_DIR="$SCENARIO/processed/jrl_${RUN_NAME}"
MESA_DIR="$SCENARIO/mesa2d/${RUN_NAME}"
TUM_OUT="$SCENARIO/eval/tum/mesa_${RUN_NAME}"
LOG_DIR="$SCENARIO/logs"

mkdir -p "$GRAPH_DIR" "$RTABMAP_DIR" "$TAG_DIR" "$DIAG_DIR" "$CONSTRAINT_DIR" "$JRL_DIR" "$MESA_DIR" "$TUM_OUT" "$LOG_DIR"

robots=()
symbols=(a b c d e f)
for idx in $(seq 1 "$ROBOT_COUNT"); do
  robots+=("robot_$(printf '%02d' "$idx")")
done

if [ "$ROBOT_COUNT" -gt "${#symbols[@]}" ]; then
  echo "This helper currently supports up to ${#symbols[@]} robots." >&2
  exit 2
fi

for robot in "${robots[@]}"; do
  if [ ! -f "$SCENARIO/raw/robots/${robot}.bag" ]; then
    echo "Missing robot bag: $SCENARIO/raw/robots/${robot}.bag" >&2
    exit 1
  fi
done

set +u
source /opt/ros/melodic/setup.bash
set -u

echo "[$(date -Is)] exporting wheel odometry trajectories"
mkdir -p "$SCENARIO/eval/tum"
for robot in "${robots[@]}"; do
  tum="$SCENARIO/eval/tum/${robot}_odom.tum"
  if [ -s "$tum" ]; then
    continue
  fi
  python tools/mesa_benchmark/export_bag_pose_tum.py \
    --bag "$SCENARIO/raw/robots/${robot}.bag" \
    --topic /odom \
    --output "$tum" \
    --max-rate 20
done

echo "[$(date -Is)] running RTAB-Map RGB-D frontends"
for robot in "${robots[@]}"; do
  graph="$GRAPH_DIR/${robot}_graph_poses.g2o"
  if [ -s "$graph" ]; then
    echo "  $robot graph exists"
    continue
  fi
  RATE="$RTABMAP_RATE" \
    SUBSCRIBE_SCAN=false \
    OUT_ROOT="$ROOT_DIR/$RTABMAP_DIR" \
    GRAPH_OUT="$ROOT_DIR/$GRAPH_DIR" \
    bash tools/mesa_benchmark/run_orkar_rtabmap_robot.sh "$SCENARIO" "$robot" \
      > "$LOG_DIR/${RUN_NAME}_${robot}_rtabmap.log" 2>&1
done

echo "[$(date -Is)] running AprilTag detections"
for robot in "${robots[@]}"; do
  det_bag="$TAG_DIR/${robot}/detections.bag"
  if [ -s "$det_bag" ]; then
    echo "  $robot detections exist"
    continue
  fi
  RATE="$APRILTAG_RATE" \
    TAG_DECIMATE="$APRILTAG_DECIMATE" \
    TAG_THREADS="$APRILTAG_THREADS" \
    TAG_SIZE="$APRILTAG_SIZE" \
    TAG_ID_MAX="$APRILTAG_ID_MAX" \
    bash tools/mesa_benchmark/run_apriltag_ros1_bag.sh "$SCENARIO" "$robot" \
      > "$LOG_DIR/${RUN_NAME}_${robot}_apriltag.log" 2>&1
done

echo "[$(date -Is)] exporting AprilTag detection CSV"
python tools/mesa_benchmark/export_apriltag_detections.py \
  --scenario "$SCENARIO" \
  --robots "$ROBOT_COUNT" \
  --output "$TAG_CSV" \
  > "$LOG_DIR/${RUN_NAME}_export_apriltags.log" 2>&1

echo "[$(date -Is)] inferring robot-mounted tag targets"
python3 tools/mesa_benchmark/orkar_apriltag_ros1_diagnostic.py \
  --detections-csv "$TAG_CSV" \
  --odom-template "$SCENARIO/eval/tum/{robot}_odom.tum" \
  --robots "${robots[@]}" \
  --out-dir "$DIAG_DIR" \
  --max-sync-s 0.50 \
  --min-used 8 \
  > "$LOG_DIR/${RUN_NAME}_tag_diagnostic.log" 2>&1

python3 tools/mesa_benchmark/write_tag_rules_from_orkar_diagnostic.py \
  --summary "$DIAG_DIR/tag_mapping_summary_ros1.json" \
  --source "$TAG_RULE_SOURCE" \
  --shift "$TAG_TARGET_SHIFT" \
  --output "$CONSTRAINT_DIR/tag_rule_args.txt"

read -r -a tag_rule_args < "$CONSTRAINT_DIR/tag_rule_args.txt"
if [ "${#tag_rule_args[@]}" -eq 0 ]; then
  echo "No AprilTag target rules were inferred. Inspect $DIAG_DIR/tag_mapping_summary_ros1.json" >&2
  exit 1
fi

echo "[$(date -Is)] generating inter-robot constraints"
python3 tools/mesa_benchmark/generate_orkar_apriltag_mesa_constraints.py \
  --scenario "$SCENARIO" \
  --detections-csv "$TAG_CSV" \
  --output-dir "$CONSTRAINT_DIR" \
  --robots "${robots[@]}" \
  --graph-dir "$GRAPH_DIR" \
  --rtabmap-dir "$RTABMAP_DIR" \
  "${tag_rule_args[@]}" \
  --max-nfev 500 \
  --fit-min-separation 0.50 \
  --fit-max-per-viewer-tag 180 \
  --max-residual 0.75 \
  --min-separation 1.0 \
  --max-per-pair 120 \
  --max-node-dt 2.0 \
  > "$LOG_DIR/${RUN_NAME}_constraints.log" 2>&1

inter_csv="$CONSTRAINT_DIR/orkar_apriltag_mesa_inter_robot_constraints.csv"
accepted="$(tail -n +2 "$inter_csv" | wc -l | tr -d ' ')"
if [ "$accepted" -lt 1 ]; then
  echo "No accepted inter-robot constraints. Inspect $LOG_DIR/${RUN_NAME}_constraints.log" >&2
  exit 1
fi

echo "[$(date -Is)] building MESA JRL dataset ($accepted inter-robot constraints)"
jrl="$JRL_DIR/${RUN_NAME}.jrl"
build_cmd=(mesa/build/experiments/multi-g2o-2-mr-jrl --force2d -n "$RUN_NAME" -o "$jrl")
for idx in "${!robots[@]}"; do
  build_cmd+=(-g "${symbols[$idx]}:$GRAPH_DIR/${robots[$idx]}_graph_poses.g2o")
done
build_cmd+=(--inter_csv "$inter_csv")
"${build_cmd[@]}" > "$LOG_DIR/${RUN_NAME}_build_jrl.log" 2>&1

echo "[$(date -Is)] running MESA"
mesa/build/experiments/run-dist-batch \
  -i "$jrl" \
  -m geodesic-mesa \
  -o "$MESA_DIR" \
  > "$LOG_DIR/${RUN_NAME}_mesa.log" 2>&1

result="$(find "$MESA_DIR" -mindepth 2 -maxdepth 2 -name final_results.jrr.cbor | sort | tail -n 1)"
if [ -z "$result" ]; then
  echo "No final MESA result found in $MESA_DIR" >&2
  exit 1
fi

db_args=()
for idx in "${!robots[@]}"; do
  db_args+=(--db-map "${symbols[$idx]}=$RTABMAP_DIR/${robots[$idx]}/rtabmap.db")
done

set +u
source .venvs/eval/bin/activate
set -u

echo "[$(date -Is)] exporting MESA result trajectories"
python tools/mesa_benchmark/export_jrr_tum.py \
  --result "$result" \
  --keymap "$jrl.keymap.csv" \
  "${db_args[@]}" \
  --output-dir "$TUM_OUT" \
  > "$LOG_DIR/${RUN_NAME}_export_mesa_tum.log" 2>&1

echo "MESA benchmark complete"
echo "  constraints: $inter_csv"
echo "  result:      $result"
echo "  TUM dir:     $TUM_OUT"
