# dataset_ground_truth

This package converts raw PhaseSpace rigid-body messages into canonical
ground-truth topics for dataset collection.

For one robot, it publishes:

- `/gt/<robot_id>/pose` (`geometry_msgs/PoseStamped`)
- `/gt/<robot_id>/odom` (`nav_msgs/Odometry`)
- `/gt/<robot_id>/path` (`nav_msgs/Path`)
- optional TF from `map` to `<robot_id>/gt_base_link`

The node keeps the raw `/phasespace/rigids` topic intact. Calibration is
expressed as `T_rigid_base`, applied after the raw mocap pose:

```text
T_map_base = T_map_rigid * T_rigid_base
```

For publication-quality runs, replace the identity transform in
`config/robot_01.yaml` with a measured rigid-body-to-robot-base transform.
