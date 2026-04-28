# dataset_validation

Initial validation tools for one-robot dataset runs.

```bash
rosrun dataset_validation validate_run.py \
  --robot_id robot_01 \
  --run_dir ~/Distributed_SLAM/runs/20260428_lab_loop_r01_run01
```

The validator writes:

- `validation/report.md`
- `validation/topic_rates.csv`

It checks required topics, topic rates, monotonic timestamps, and zero message
header stamps.
