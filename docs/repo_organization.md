# Repository Organization

This repo should be easy to review because each directory has one job.

## Ownership Boundaries

| Path | Owner | Purpose |
| --- | --- | --- |
| `catkin_ws/src/dataset_ground_truth/` | This repo | PhaseSpace timestamping, rigid-body GT, marker debug fallback |
| `catkin_ws/src/dataset_recording/` | This repo | Dataset run folders, rosbag capture, Chrony/NTP snapshots |
| `catkin_ws/src/dataset_validation/` | This repo | Pass/fail checks for recorded runs |
| `catkin_ws/src/robot_description/` | This repo | URDF, meshes, sensor frames, calibration RViz |
| `catkin_ws/src/slam_comparison/` | This repo | Local comparison utilities and plotting |
| `external/robot_runtime/` | Submodule | Robot-side bringup, drivers, robot-local logging |
| `external/agv_on_board/` | Submodule | Alternate robot-side codebase; should be merged or retired later |
| `external/offline_slam_pipeline/` | Submodule | Offline SLAM algorithms/evaluation |
| `external/phasespace_mocap_ros/` | Submodule | Upstream PhaseSpace ROS bridge; keep local dataset fixes outside it |
| `external/ydlidar_sdk/` | Submodule | Upstream YDLidar SDK used by lidar driver builds |
| `catkin_ws/src/phasespace_bringup` | Symlink | ROS-visible link into `external/phasespace_mocap_ros` |
| `catkin_ws/src/phasespace_msgs` | Symlink | ROS-visible link into `external/phasespace_mocap_ros` |
| `catkin_ws/src/YDLidar-SDK` | Symlink | ROS/build-visible link into `external/ydlidar_sdk` |
| `runs/` | Local only | Raw bags, robot uploads, validation outputs, processed artifacts |

## Current External Layout

Submodules are centralized under `external/`:

```text
external/
  robot_runtime/          current robot runtime submodule
  agv_on_board/           alternate robot runtime submodule
  offline_slam_pipeline/  offline SLAM/evaluation submodule
  phasespace_mocap_ros/   upstream PhaseSpace bridge submodule
  ydlidar_sdk/            upstream YDLidar SDK submodule

catkin_ws/src/
  dataset_ground_truth/
  dataset_recording/
  dataset_validation/
  robot_description/
  slam_comparison/

docs/
  experiment_workflow.md
  repo_organization.md
  calibration/
  operations/
```

Do not edit submodule source from the parent repo. If a submodule needs code
changes, commit those changes in that submodule repository first.

## What Not To Commit

- `runs/`
- `*.bag`, `*.bag.active`, `*.db3`, `*.mcap`
- `catkin_ws/build/`, `catkin_ws/devel/`, `catkin_ws/install/`
- generated `*.pyc`, `__pycache__/`
- Windows `Zone.Identifier` files
- one-off plots unless they are intentionally small validation artifacts

## Publication Rule

Every dataset run should be reproducible from:

1. The parent repo commit.
2. All submodule SHAs.
3. The run metadata file.
4. Calibration files used to generate `/gt/<robot_id>/*`.
5. Raw mocap and robot bags.
