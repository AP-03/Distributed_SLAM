# One-Robot Dataset Workflow

This repository is organized around dataset acquisition rather than online
SLAM. The robot drives a scenario and records raw sensor data; the lab station
records mocap ground truth; SLAM algorithms run offline after the run has been
validated.

## Package Roles

- `external/phasespace_mocap_ros`: raw PhaseSpace bridge. Publishes `/phasespace/*`.
- `dataset_ground_truth`: converts PhaseSpace rigid bodies, or the temporary
  8-LED marker bootstrap fallback, into canonical ground-truth topics under
  `/gt/<robot_id>/*`.
- `dataset_recording`: creates run directories, captures NTP diagnostics, and
  runs `rosbag record`.
- `dataset_validation`: checks a completed run for required topics and
  timestamp issues.
- `slam_comparison`: RViz/debug comparison and offline odometry-vs-GT plots.
- `robot_description`: robot frames, meshes, and calibrated sensor transforms.

## Run Directory

Every experiment should have a unique immutable `run_id`:

```text
YYYYMMDD_site_scenario_rXX_runNN
20260428_lab_loop_r01_run01
```

The recorder creates:

```text
runs/<run_id>/
  raw/
    robot_01.bag
  calibration/
  validation/
    ntp_status.txt
    report.md
    topic_rates.csv
  processed/
  metadata.yaml
```

Raw bags are never edited. Alignment, exports, and plots belong in
`processed/`.

## One-Robot Collection

1. Start the robot-side bringup so `/robot_01/odom`, `/robot_01/scan`, `/tf`,
   and `/tf_static` are available.
2. Confirm either the mocap rigid-body ID or, for the temporary pilot setup,
   that marker IDs `8..15` are mounted on the robot and visible.
3. Run the collection launch from the lab station:

```bash
roslaunch dataset_recording collect_one_robot.launch \
  robot_id:=robot_01 \
  scenario:=loop \
  run_id:=20260428_lab_loop_r01_run01 \
  server_ip:=192.168.1.230 \
  notes:="pilot square loop"
```

4. Leave the robot stationary for 5-10 seconds.
5. Drive the planned path.
6. Leave the robot stationary for 5-10 seconds.
7. Stop recording with `Ctrl-C`.
8. Validate:

```bash
rosrun dataset_validation validate_run.py \
  --robot_id robot_01 \
  --run_dir ~/Distributed_SLAM/runs/20260428_lab_loop_r01_run01 \
  --profile one_robot_marker_bootstrap
```

## Topic Contract

For the one-robot pilot, the expected topics are:

```text
/phasespace/rigids
/phasespace/stamped/rigids
/phasespace/stamped/markers
/gt/robot_01/pose
/gt/robot_01/odom
/gt/robot_01/path
/robot_01/odom
/robot_01/scan
/robot_01/imu
/robot_01/camera/color/image_raw
/robot_01/camera/color/camera_info
/robot_01/camera/depth/image_rect_raw
/robot_01/camera/depth/camera_info
/tf
/tf_static
```

It is fine if the first pilot records only odom, lidar, TF, and mocap. The
validator will make missing streams obvious.

## Ground Truth Calibration

The initial ground-truth config uses an identity transform from the PhaseSpace
rigid body to the robot base:

```text
T_map_base = T_map_rigid * T_rigid_base
```

Before publication-quality collection, measure and save `T_rigid_base` in
`dataset_ground_truth/config/robot_01.yaml`. First-pose alignment is useful for
debugging, but the released dataset should use a measured calibration.

For this week's pilot, `robot_01_marker_bootstrap.yaml` defines a temporary
8-LED cluster. It bootstraps the marker geometry from the first valid frame and
uses the LED centroid as the GT origin. This is acceptable for checking that
mocap, odom, lidar, camera recording, and offline SLAM evaluation all connect,
but it should not be used as final dataset ground truth.

## Multi-Robot Mocap Collection

Preferred setup: define one rigid body per robot inside the PhaseSpace software
using that robot's 8 stable LEDs. Then list those PhaseSpace rigid-body IDs in
`dataset_ground_truth/config/multi_robot_rigids.yaml`.

The lab station records one mocap bag for all robots:

```bash
roslaunch dataset_recording record_mocap_multi_robot.launch \
  run_id:=20260429_lab_multirobot_r01_run01 \
  server_ip:=192.168.1.25 \
  show_rviz:=true
```

This launch records raw `/phasespace/*`, stamped `/phasespace/stamped/*`,
`/gt/<robot_id>/*`, `/tf`, `/tf_static`, and the live debug visualization
topics. Each robot should still record its own local sensor bag during the same
run. Offline dataset assembly should align the mocap bag and robot bags by ROS
timestamps after clock sync.

If `rigid_to_base_xyz/rpy` is still identity, the GT pose is the PhaseSpace
rigid-body origin. That is okay for collection as long as raw mocap is saved;
after measurement, replay the raw mocap bag with the calibrated
`multi_robot_rigids.yaml` to regenerate calibrated `/gt/<robot_id>/*` topics.

## PhaseSpace Driver Boundary

`external/phasespace_mocap_ros` is kept as an upstream submodule. Dataset-critical
timestamping and frame naming live in `dataset_ground_truth` instead:

```text
/phasespace/markers          raw upstream driver output
/phasespace/stamped/markers  dataset-safe stamped output
/phasespace/rigids           raw upstream driver output
/phasespace/stamped/rigids   dataset-safe stamped output
```

Use the stamped topics for ground truth, RViz validation, and dataset pass/fail
checks. Recording configs keep the raw topics for audit, but validation should
require the stamped topics. This prevents a future submodule reset from wiping
our dataset timestamp/frame behavior.

## Acceptance Criteria For This Week

A one-robot pilot run is accepted when:

- PhaseSpace raw data is bridged to stamped `/phasespace/stamped/markers`;
  `/phasespace/stamped/rigids` is preferred once the rigid body is configured.
- `/gt/robot_01/pose` follows the robot in RViz.
- The robot bag contains `/robot_01/odom`, `/robot_01/scan`, `/tf`, and
  `/tf_static`.
- Mocap and robot data overlap in time.
- `dataset_validation` produces `PASS` or only known, documented warnings.
- A quick trajectory plot from `slam_comparison` is plausible.
