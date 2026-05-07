# dataset_recording

This package creates a repeatable run directory and records the expected
one-robot dataset topics.

Typical use:

```bash
roslaunch dataset_recording collect_one_robot.launch \
  robot_id:=robot_01 \
  scenario:=loop \
  run_id:=20260428_lab_loop_r01_run01 \
  server_ip:=192.168.1.230
```

The recorder creates:

```text
runs/<run_id>/
  raw/<robot_id>.bag
  calibration/
  validation/ntp_status.txt
  processed/
  metadata.yaml
```

Start the robot-side bringup separately until the robot code is merged into
this repository. Missing topics are visible immediately in the recorder output
and will be flagged by `dataset_validation`.

## Mocap-Only Debug Recording

To wave loose LEDs through the capture space and record the raw mocap stream:

```bash
roslaunch dataset_recording record_mocap_debug.launch \
  run_id:=20260429_mocap_led_wave_01 \
  server_ip:=192.168.1.230
```

The bag is written to:

```text
~/Distributed_SLAM/runs/<run_id>/raw/mocap_debug.bag
```

Replay it with:

```bash
roslaunch dataset_ground_truth mocap_debug_playback.launch \
  bag:=$HOME/Distributed_SLAM/runs/<run_id>/raw/mocap_debug.bag \
  loop:=true
```

## Mocap-Only Dataset Capture

Use this when the robot records its own data locally and this machine is only
responsible for mocap:

```bash
roslaunch dataset_recording record_mocap_only.launch \
  run_id:=20260429_lab_loop_r01_run01 \
  server_ip:=192.168.1.25 \
  robot_id:=robot_01
```

By default this uses the temporary 8-LED bootstrap GT from marker IDs `8..15`
and records `/gt/robot_01/{pose,odom,path,status}`. To switch back to a
PhaseSpace server-defined rigid body later:

```bash
roslaunch dataset_recording record_mocap_only.launch \
  run_id:=20260429_lab_loop_r01_run01 \
  server_ip:=192.168.1.25 \
  robot_id:=robot_01 \
  use_marker_bootstrap_gt:=false
```

The bag is written to:

```text
~/Distributed_SLAM/runs/<run_id>/raw/mocap.bag
```

After the run, copy the robot's local bag into the same run directory:

```text
~/Distributed_SLAM/runs/<run_id>/raw/robot_01.bag
```
