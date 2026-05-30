#!/usr/bin/env python3
"""ORKAR-style GT time-shift and label sweep for local TUM trajectories.

This is a local equivalent of ORKAR's `sweep_visual_gt_time_shift.py`, using
already-exported TUM files instead of reading ROS bags through `rosbags`.
It is an evaluation utility only: it does not build SLAM factors and it does
not feed ground truth into MESA.
"""

import argparse
import csv
import itertools
import json
import math
from pathlib import Path

import numpy as np


def yaw_from_quat(qx, qy, qz, qw):
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def read_tum(path):
    rows = []
    with open(path) as handle:
        for line in handle:
            parts = line.split()
            if len(parts) < 8:
                continue
            rows.append([float(x) for x in parts[:8]])
    if not rows:
        raise RuntimeError("empty TUM trajectory: %s" % path)
    return np.asarray(rows, dtype=float)


def shift_values(min_shift, max_shift, step):
    if step <= 0.0:
        raise ValueError("--shift-step must be positive")
    count = int(np.floor((max_shift - min_shift) / step + 0.5))
    values = [min_shift + idx * step for idx in range(count + 1)]
    if not values or values[-1] < max_shift - 1e-9:
        values.append(max_shift)
    return [round(float(v), 9) for v in values]


def thin_indices(n, max_poses):
    if max_poses <= 0 or n <= max_poses:
        return np.arange(n, dtype=int)
    return np.linspace(0, n - 1, max_poses).astype(int)


def interpolate_gt(gt, query_times):
    gt_t = gt[:, 0]
    keep = (query_times >= gt_t[0]) & (query_times <= gt_t[-1])
    kept = query_times[keep]
    if len(kept) == 0:
        return np.empty((0, 3), dtype=float), keep
    xyz = np.column_stack([np.interp(kept, gt_t, gt[:, axis]) for axis in (1, 2, 3)])
    return xyz, keep


def fit_se2(src_xy, dst_xy):
    src_mu = src_xy.mean(axis=0)
    dst_mu = dst_xy.mean(axis=0)
    src_c = src_xy - src_mu
    dst_c = dst_xy - dst_mu
    c = float(np.sum(src_c[:, 0] * dst_c[:, 0] + src_c[:, 1] * dst_c[:, 1]))
    s = float(np.sum(src_c[:, 0] * dst_c[:, 1] - src_c[:, 1] * dst_c[:, 0]))
    theta = math.atan2(s, c)
    ct = math.cos(theta)
    st = math.sin(theta)
    rot = np.asarray([[ct, -st], [st, ct]], dtype=float)
    trans = dst_mu - (rot @ src_mu)
    return rot, trans


def fit_sim2(src_xy, dst_xy):
    src_mu = src_xy.mean(axis=0)
    dst_mu = dst_xy.mean(axis=0)
    src_c = src_xy - src_mu
    dst_c = dst_xy - dst_mu
    c = float(np.sum(src_c[:, 0] * dst_c[:, 0] + src_c[:, 1] * dst_c[:, 1]))
    s = float(np.sum(src_c[:, 0] * dst_c[:, 1] - src_c[:, 1] * dst_c[:, 0]))
    theta = math.atan2(s, c)
    ct = math.cos(theta)
    st = math.sin(theta)
    rot = np.asarray([[ct, -st], [st, ct]], dtype=float)
    denom = float(np.sum(src_c * src_c))
    scale = math.sqrt(c * c + s * s) / denom if denom > 1e-12 else 1.0
    trans = dst_mu - scale * (rot @ src_mu)
    return scale, rot, trans


def stats(err):
    return {
        "rmse_m": float(np.sqrt(np.mean(err * err))),
        "median_m": float(np.median(err)),
        "p95_m": float(np.percentile(err, 95)),
        "max_m": float(np.max(err)),
    }


def metric(reference_xyz, estimate_xyz):
    ref_xy = reference_xyz[:, :2]
    est_xy = estimate_xyz[:, :2]
    rot, trans = fit_se2(est_xy, ref_xy)
    aligned = est_xy @ rot.T + trans
    se2_err = np.linalg.norm(aligned - ref_xy, axis=1)

    scale, srot, strans = fit_sim2(est_xy, ref_xy)
    saligned = scale * (est_xy @ srot.T) + strans
    sim2_err = np.linalg.norm(saligned - ref_xy, axis=1)
    out = {
        "se2": stats(se2_err),
        "sim2": stats(sim2_err),
        "sim2_scale": float(scale),
    }
    return out


def evaluate_pair(est, gt, shifts, min_duration, min_poses, max_poses):
    rows = []
    if max_poses > 0 and len(est) > max_poses:
        est_eval = est[thin_indices(len(est), max_poses)]
    else:
        est_eval = est
    for shift in shifts:
        shifted_gt = gt.copy()
        shifted_gt[:, 0] = gt[:, 0] - shift
        reference, keep = interpolate_gt(shifted_gt, est_eval[:, 0])
        if len(reference) < min_poses:
            continue
        matched_est = est_eval[keep]
        duration = float(matched_est[-1, 0] - matched_est[0, 0])
        if duration < min_duration:
            continue
        m = metric(reference, matched_est[:, 1:4])
        rows.append(
            {
                "gt_time_shift_s": shift,
                "matched_poses": int(len(reference)),
                "duration_s": duration,
                "se2_rmse_m": m["se2"]["rmse_m"],
                "se2_median_m": m["se2"]["median_m"],
                "se2_p95_m": m["se2"]["p95_m"],
                "se2_max_m": m["se2"]["max_m"],
                "sim2_rmse_m": m["sim2"]["rmse_m"],
                "sim2_median_m": m["sim2"]["median_m"],
                "sim2_p95_m": m["sim2"]["p95_m"],
                "sim2_scale": m["sim2_scale"],
            }
        )
    rows.sort(key=lambda row: (row["se2_rmse_m"], row["gt_time_shift_s"]))
    return rows


def write_csv(path, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        path.write_text("")
        return
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def best_assignment(robots, gt_labels, best_rows):
    if len(gt_labels) < len(robots):
        return None
    assignments = []
    for perm in itertools.permutations(gt_labels, len(robots)):
        selected = []
        ok = True
        for robot, gt in zip(robots, perm):
            row = best_rows.get((robot, gt))
            if row is None:
                ok = False
                break
            selected.append(row)
        if not ok:
            continue
        assignments.append(
            {
                "mapping": {robot: gt for robot, gt in zip(robots, perm)},
                "mean_se2_rmse_m": float(np.mean([row["se2_rmse_m"] for row in selected])),
                "mean_sim2_rmse_m": float(np.mean([row["sim2_rmse_m"] for row in selected])),
                "rows": selected,
            }
        )
    assignments.sort(key=lambda row: (row["mean_se2_rmse_m"], row["mean_sim2_rmse_m"]))
    return assignments[0] if assignments else None


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--estimate-template", required=True, help="Template with {robot}.")
    parser.add_argument("--gt-template", required=True, help="Template with {gt}.")
    parser.add_argument("--robots", nargs="+", required=True)
    parser.add_argument("--gt-labels", nargs="+", required=True)
    parser.add_argument("--shift-min", type=float, default=-240.0)
    parser.add_argument("--shift-max", type=float, default=240.0)
    parser.add_argument("--shift-step", type=float, default=1.0)
    parser.add_argument("--min-duration", type=float, default=120.0)
    parser.add_argument("--min-poses", type=int, default=60)
    parser.add_argument(
        "--max-poses",
        type=int,
        default=0,
        help="Optional deterministic thinning of each estimate; 0 uses all poses.",
    )
    parser.add_argument("--out-dir", required=True)
    parser.add_argument("--name", required=True)
    return parser.parse_args()


def main():
    args = parse_args()
    out_dir = Path(args.out_dir)
    shifts = shift_values(args.shift_min, args.shift_max, args.shift_step)
    estimates = {
        robot: read_tum(args.estimate_template.format(robot=robot))
        for robot in args.robots
    }
    gts = {
        gt: read_tum(args.gt_template.format(gt=gt))
        for gt in args.gt_labels
    }

    all_rows = []
    best_rows = {}
    for robot in args.robots:
        for gt in args.gt_labels:
            rows = evaluate_pair(
                estimates[robot],
                gts[gt],
                shifts,
                args.min_duration,
                args.min_poses,
                args.max_poses,
            )
            for row in rows:
                out = {"method": args.name, "robot": robot, "gt": gt}
                out.update(row)
                all_rows.append(out)
            if rows:
                best = {"method": args.name, "robot": robot, "gt": gt}
                best.update(rows[0])
                best_rows[(robot, gt)] = best

    all_rows.sort(key=lambda row: (row["robot"], row["se2_rmse_m"], row["gt"], row["gt_time_shift_s"]))
    best_table = sorted(best_rows.values(), key=lambda row: (row["robot"], row["se2_rmse_m"]))
    assignment = best_assignment(args.robots, args.gt_labels, best_rows)

    write_csv(out_dir / ("%s_time_shift_sweep.csv" % args.name), all_rows)
    write_csv(out_dir / ("%s_best_per_robot_gt.csv" % args.name), best_table)
    payload = {
        "method": args.name,
        "estimate_template": args.estimate_template,
        "gt_template": args.gt_template,
        "robots": args.robots,
        "gt_labels": args.gt_labels,
        "shift_min": args.shift_min,
        "shift_max": args.shift_max,
        "shift_step": args.shift_step,
        "min_duration": args.min_duration,
        "min_poses": args.min_poses,
        "max_poses": args.max_poses,
        "best_assignment": assignment,
        "best_rows": best_table,
    }
    summary_path = out_dir / ("%s_time_shift_summary.json" % args.name)
    summary_path.write_text(json.dumps(payload, indent=2, allow_nan=False) + "\n")
    print(json.dumps({"method": args.name, "best_assignment": assignment}, indent=2, allow_nan=False))
    print("wrote %s" % summary_path)


if __name__ == "__main__":
    main()
