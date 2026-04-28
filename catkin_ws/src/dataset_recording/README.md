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
