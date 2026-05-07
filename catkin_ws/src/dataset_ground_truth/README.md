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

## Temporary 8-LED Cluster GT

For the current pilot robot, PhaseSpace is publishing loose LEDs but not a
server-defined rigid body. Use the temporary bootstrap node to treat LEDs
`8..15` as one cluster:

```bash
roslaunch dataset_ground_truth marker_bootstrap_ground_truth.launch
```

It publishes the same `/gt/<robot_id>/{pose,odom,path}` topics. The first good
frame defines the LED-cluster geometry and the GT origin is the LED centroid,
not `base_link`, so this is only for pipeline testing and relative comparisons.
Use measured marker positions or a PhaseSpace-defined rigid body for final data.

## Mocap Debugging

For loose LEDs and rigid-body smoke tests:

```bash
roslaunch dataset_ground_truth mocap_debug_live.launch \
  server_ip:=192.168.1.230 \
  frame_id:=mocap_world
```

This opens RViz with:

- colored LED spheres and labels from `/phasespace/markers`
- per-LED trails from `/mocap_debug/led_paths`
- rigid-body arrows and trails from `/phasespace/rigids`

After recording a bag, replay it with:

```bash
roslaunch dataset_ground_truth mocap_debug_playback.launch \
  bag:=/path/to/mocap_debug.bag \
  start:=0.0 \
  loop:=true
```

The playback launch also starts the temporary 8-LED bootstrap GT by default, so
old loose-marker bags can be replayed into `/gt/robot_01/*`.

Summarize marker IDs and path lengths:

```bash
rosrun dataset_ground_truth mocap_debug_summary.py \
  --bag /path/to/mocap_debug.bag
```
