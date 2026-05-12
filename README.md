# Distributed SLAM Dataset Stack

This repository is the integration repo for the distributed SLAM dataset
project. It owns the mocap ground-truth, dataset recording, validation,
calibration, and experiment documentation needed to collect publishable
multi-robot SLAM data.

The robot runtime and offline SLAM implementations are kept as submodules so
they can evolve in their own repositories while this repo pins the exact
versions used for each dataset release.

## What Lives Where

```text
catkin_ws/src/
  dataset_ground_truth/   PhaseSpace -> /gt/<robot_id> pose/odom/path
  dataset_recording/      run folders, rosbag capture, NTP/Chrony snapshots
  dataset_validation/     bag/topic/timestamp checks for completed runs
  robot_description/      URDF, meshes, RViz calibration views, sensor frames
  slam_comparison/        local comparison/plotting helpers

external/
  robot_runtime/            robot-side runtime submodule
  agv_on_board/             alternate/partner robot-side repo submodule
  offline_slam_pipeline/    offline SLAM/evaluation submodule
  phasespace_mocap_ros/     upstream PhaseSpace ROS bridge submodule
  ydlidar_sdk/              YDLidar SDK submodule

catkin_ws/src/phasespace_* and catkin_ws/src/YDLidar-SDK are symlinks into
external/ so ROS tools can still find the packages/SDK where they expect them.

docs/                       workflow, calibration, and repo policy docs
runs/                       local experiment outputs; never committed
```

## Main Workflows

Mocap-only collection on the lab station:

```bash
cd ~/Distributed_SLAM/catkin_ws
source devel/setup.bash
roslaunch dataset_recording record_mocap_multi_robot.launch \
  run_id:=YYYYMMDD_site_scenario_rXX_runNN \
  server_ip:=192.168.1.25 \
  show_rviz:=true
```

One-robot pilot validation:

```bash
rosrun dataset_validation validate_run.py \
  --robot_id robot_01 \
  --run_dir ~/Distributed_SLAM/runs/YYYYMMDD_site_scenario_rXX_runNN
```

Robot-side collection is handled in the robot runtime submodule. The normal
pattern is:

1. Start mocap recording on the lab station.
2. Start robot-local recording on each robot.
3. Drive the planned scenario.
4. Stop recordings and copy robot bags into the matching `runs/<run_id>/`.
5. Validate, align, and process offline.

## Submodule Policy

Submodules are treated as pinned dependencies. Do not edit submodule code from
the parent repo unless you intend to make a commit in that submodule too.

Use this when cloning:

```bash
git clone --recurse-submodules https://github.com/AP-03/Distributed_SLAM.git
```

Use this when updating pinned submodule versions:

```bash
git submodule update --init --recursive
git -C external/robot_runtime status
git -C external/offline_slam_pipeline status
```

If a submodule has conflicts or local edits, resolve and commit those inside the
submodule first, then commit the updated submodule pointer in this parent repo.

## Documentation

- [Experiment workflow](docs/experiment_workflow.md)
- [Repository organization](docs/repo_organization.md)
- [Cleanup status](docs/cleanup_status.md)
- [Contribution and PR rules](CONTRIBUTING.md)

## Data Policy

Raw datasets, bags, generated plots, build folders, and local logs stay out of
git. The repo should contain code, configuration, calibration files,
documentation, and small validation reports only.
