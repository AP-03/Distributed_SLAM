# Contributing

Use pull requests for any change that affects robot runtime, mocap collection,
ground truth, validation, offline evaluation, or dataset release artifacts.

## Branches

Use descriptive branches:

```text
dataset/mocap-multi-rigid
robot/cmd-vel-watchdog
validation/topic-timing-report
docs/calibration-sop
```

## Pull Request Checklist

Before requesting review:

- State which workflow the change affects: mocap, robot runtime, validation,
  offline SLAM, docs, or calibration.
- Include exact commands used for build/test/validation.
- Mention whether any submodule pointer changed.
- Do not include raw bags or generated build files.
- For robot motion changes, describe safety behavior and stop conditions.
- For ground-truth changes, describe frame conventions and timestamp source.

## Submodule Changes

If you edit a submodule:

1. Commit the change inside the submodule repo.
2. Push or open a PR in that submodule repo.
3. Return to this parent repo.
4. Commit the updated submodule pointer here.

Do not leave submodules mid-merge in the parent repo. Reviewers cannot safely
reason about a parent PR when a child repo has unresolved conflicts.

## Review Rules

High-risk changes need extra care:

- Robot motion/control: require a bench test or low-speed supervised test.
- Dataset recording: require a tiny dry-run bag and validation output.
- Ground truth/calibration: require a replay or RViz screenshot/notes.
- Offline SLAM claims: require input topics and metric definitions.

## Commit Hygiene

Keep commits small and role-based. Prefer:

```text
Add multi-rigid mocap GT node
Document PhaseSpace timestamp boundary
Add robot cmd_vel watchdog
```

Avoid mixed commits that touch robot drivers, offline SLAM, and docs at the
same time unless the change is intentionally coordinated.
