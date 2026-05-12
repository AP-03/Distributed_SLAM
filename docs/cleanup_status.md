# Cleanup Status

This file tracks repository hygiene issues that should be fixed before large
renames, submodule moves, or release tagging.

## Current Blockers

### `external/robot_runtime` is mid-merge

The robot runtime submodule is currently ahead/behind its remote and has
unresolved conflicts in:

```text
agv_ws/src/agv_bringup/launch/bringup.launch
scripts/logging/start_session.sh
scripts/setup_robot.sh
```

Resolve this inside `external/robot_runtime`, commit it there, then commit the
updated submodule pointer in the parent repo.

### `external/agv_on_board` has broken nested submodule metadata

`external/agv_on_board` contains gitlink entries for:

```text
drivers/librealsense
drivers/teleop_twist_keyboard
```

but does not contain matching `.gitmodules` entries. Because of that,
`git submodule status --recursive` fails when it reaches `external/agv_on_board`.

Either add the missing nested `.gitmodules` entries in `external/agv_on_board`,
remove those gitlinks if they are obsolete, or retire this duplicate robot-side
submodule after confirming `external/robot_runtime` is the source of truth.

## Recommended Cleanup Order

1. Resolve and commit `external/robot_runtime`.
2. Decide whether `external/agv_on_board` is still needed.
3. Fix or retire `external/agv_on_board` nested submodules.
4. Commit the parent repo with clean submodule pointers.
5. Remove or archive duplicate robot runtime code once one source of truth is chosen.
