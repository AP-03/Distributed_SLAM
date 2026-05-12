# Setup

This repo is the dataset/mocap integration workspace. Robot runtime and offline
SLAM code live in submodules under `external/`.

## Clone

```bash
git clone --recurse-submodules https://github.com/AP-03/Distributed_SLAM.git
cd Distributed_SLAM
```

If you already cloned without submodules:

```bash
git submodule update --init --recursive
```

## Ubuntu / ROS

The lab station currently uses Ubuntu 18.04 / ROS Melodic. In WSL, use the
`Ubuntu-18.04-Melodic` distro.

Install the basic dependencies:

```bash
sudo apt update
sudo apt install -y \
  build-essential \
  python-catkin-tools \
  python-numpy \
  python-yaml \
  chrony \
  ros-melodic-desktop \
  ros-melodic-rosbag \
  ros-melodic-tf \
  ros-melodic-tf2-ros \
  ros-melodic-robot-state-publisher \
  ros-melodic-rviz
```

## Build

```bash
cd ~/Distributed_SLAM/catkin_ws
source /opt/ros/melodic/setup.bash
catkin_make
source devel/setup.bash
```

The PhaseSpace packages are exposed to catkin through symlinks:

```text
catkin_ws/src/phasespace_bringup -> ../../external/phasespace_mocap_ros/phasespace_bringup
catkin_ws/src/phasespace_msgs    -> ../../external/phasespace_mocap_ros/phasespace_msgs
catkin_ws/src/YDLidar-SDK        -> ../../external/ydlidar_sdk
```

## Verify

```bash
rospack find dataset_ground_truth
rospack find dataset_recording
rospack find dataset_validation
rospack find phasespace_bringup
```

## Mocap Recording

```bash
cd ~/Distributed_SLAM/catkin_ws
source devel/setup.bash
roslaunch dataset_recording record_mocap_multi_robot.launch \
  run_id:=YYYYMMDD_site_scenario_rXX_runNN \
  server_ip:=192.168.1.25 \
  show_rviz:=true
```

## Robot Runtime

Robot-side bringup, local logging, and motion scripts are in:

```text
external/robot_runtime/
```

That submodule currently has unresolved merge conflicts, so resolve it inside
the submodule before using it for robot deployment.

## Data

Experiment outputs belong under:

```text
runs/<run_id>/
```

Raw bags and generated outputs are ignored by git.
