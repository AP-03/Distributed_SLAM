# Benchmarking Method

This project evaluates MESA as a distributed backend for multi-robot
pose-graph SLAM. The benchmark is offline and file-based: each robot records a
local ROS bag, the lab station records mocap ground truth, and all frontend,
backend, and evaluation steps run after collection.

## Reportable Pipeline

1. Export robot wheel odometry and mocap ground truth to TUM trajectories.
2. Run RTAB-Map RGB-D independently on each robot bag to build local frontend
   pose graphs.
3. Run AprilTag detection on each robot bag.
4. Infer robot-mounted tag targets from odometry/tag consistency. This uses
   robot odometry and tag observations only; mocap is not used.
5. Convert accepted AprilTag detections into inter-robot pose constraints.
6. Build the multi-robot JRL dataset and run MESA.
7. Export MESA results to TUM and evaluate against mocap with EVO.

The main entry point is:

```bash
RTABMAP_RATE=3 APRILTAG_RATE=4 \
  bash tools/mesa_benchmark/run_mesa_frontend_pipeline.sh \
  runs/<scenario> <robot_count> <run_name>
```

## Ground Truth Use

Mocap ground truth is used for evaluation and for estimating the
robot-bag-to-mocap-label/time mapping when robot clocks were not synchronized.
It is not used to generate frontend factors, inter-robot constraints, or MESA
input graphs.

If a dataset has pickup events or impossible mocap jumps, those windows should
be detected with `diagnose_mocap_motion.py` and excluded from evaluation with a
written reason.

## Time Alignment

Some datasets were collected without reliable Chrony synchronization between
the robots and mocap machine. For those runs, label and clock offsets are
estimated from odometry-to-mocap trajectory consistency and saved as CSV/JSON
under `runs/<scenario>/eval/`. The same mapping file must be reused for all
methods in the scenario.

## What Is Not Reportable

The following are not valid benchmark evidence for the final comparison:

- inter-robot constraints generated directly from mocap or ground truth;
- path-shape-only metrics that ignore timestamps;
- odometry-only MESA runs presented as frontend-fed distributed SLAM;
- manually shifted plots that do not correspond to the recorded evaluation
  mapping;
- solar/circle figures where the backend did not receive AprilTag or visual
  inter-robot constraints.

Those diagnostics can be useful during development, but they should not appear
in the submitted results table.

## Output Checklist

For each submitted scenario, keep:

- local RTAB-Map graph files and databases;
- AprilTag detections CSV;
- inter-robot constraint CSV and diagnostic summary;
- MESA result directory and exported TUM files;
- mapping CSV used for mocap evaluation;
- EVO result zips and CSV summaries;
- one XY plot showing MESA and mocap in the same frame;
- a short note on skipped robots, excluded mocap windows, or failed sensors.
