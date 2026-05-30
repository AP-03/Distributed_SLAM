# Cleanup Status

This file records repository hygiene issues that affect reproducibility or
release packaging.

## Submission State

The ROS dataset packages and benchmark helper scripts are organized for the
coursework submission. Local datasets, build folders, virtual environments, and
third-party checkouts are ignored by git.

The benchmark tooling has been narrowed to the defensible pipeline described in
[`benchmarking_method.md`](benchmarking_method.md). Helpers for odometry-only
MESA plots, path-shape metrics, and ground-truth/oracle constraints have been
removed from the reportable toolset.

## Remaining External Repository Issues

### `external/agv_on_board`

Recursive submodule status still fails inside `external/agv_on_board` because
that repository contains nested gitlinks without matching `.gitmodules`
entries:

```text
drivers/librealsense
drivers/teleop_twist_keyboard
```

This is an upstream/submodule hygiene issue, not a dataset package issue. Do
not edit around it in the parent repository. Either fix the nested submodule
metadata inside `external/agv_on_board` or retire that duplicate robot-side
repository after confirming `external/robot_runtime` is the source of truth.

### Robot Runtime Working Tree

The robot runtime submodule contains local robot-side logging changes. Those
changes should be committed inside the relevant submodule before updating the
parent repository pointer.

## Pre-Submission Checklist

- `git status --short` should show only intended source/doc/config changes.
- `runs/`, `deps/`, `.venvs/`, local MESA builds, and generated plots should
  not be committed.
- `tools/mesa_benchmark/run_mesa_frontend_pipeline.sh` should be the only
  frontend-to-MESA benchmark entry point used for reportable results.
- EVO tables should be traceable to saved TUM trajectories and a mapping CSV.
- Any mocap exclusion windows should have a written diagnostic reason.
