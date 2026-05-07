# dataset_validation

Initial validation tools for one-robot dataset runs.

```bash
rosrun dataset_validation validate_run.py \
  --robot_id robot_01 \
  --run_dir ~/Distributed_SLAM/runs/20260428_lab_loop_r01_run01 \
  --profile one_robot_marker_bootstrap
```

The validator writes:

- `validation/report.md`
- `validation/topic_rates.csv`

It checks required topics, topic rates, monotonic timestamps, and zero message
header stamps.

Profiles:

- `mocap_only_marker_bootstrap`: current mocap laptop run, loose LEDs `8..15`
  converted into `/gt/robot_01/*`.
- `one_robot_marker_bootstrap`: merged robot+mocap bag with robot odom and
  temporary marker GT.
- `mocap_only_rigid` / `one_robot_rigid`: use these after PhaseSpace publishes
  `/phasespace/rigids`.
