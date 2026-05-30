#!/usr/bin/env python3
import argparse
import csv
import math
import os
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


REPORT_COLORS = [
    "#1f77b4",
    "#ff7f0e",
    "#2ca02c",
    "#d62728",
    "#9467bd",
    "#8c564b",
]


def read_tum(path):
    rows = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) >= 8:
                rows.append([float(x) for x in parts[:8]])
    if not rows:
        raise RuntimeError("empty TUM trajectory: %s" % path)
    return np.asarray(rows, dtype=float)


def write_tum(path, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w") as f:
        for row in rows:
            f.write("%.9f %.9f %.9f %.9f %.9f %.9f %.9f %.9f\n" % tuple(row))


def load_mapping(path):
    with open(path) as f:
        return list(csv.DictReader(f))


def load_exclusion_windows(path, min_duration, reason_tokens):
    if not path or not os.path.exists(path):
        return {}
    tokens = [token.strip() for token in reason_tokens.split(",") if token.strip()]
    windows = {}
    with open(path) as f:
        for row in csv.DictReader(f):
            duration = float(row.get("duration_s", 0.0))
            reasons = row.get("reasons", "")
            if duration < min_duration:
                continue
            if tokens and not any(token in reasons for token in tokens):
                continue
            windows.setdefault(row["label"], []).append((float(row["start_s"]), float(row["end_s"])))
    return windows


def not_excluded(times, windows):
    keep = np.ones(len(times), dtype=bool)
    for start, end in windows:
        keep &= ~((times >= start) & (times <= end))
    return keep


def interp_xyz(gt, query_times):
    gt_t = gt[:, 0]
    keep = (query_times >= gt_t[0]) & (query_times <= gt_t[-1])
    q = query_times[keep]
    xyz = np.column_stack([np.interp(q, gt_t, gt[:, i]) for i in (1, 2, 3)])
    return q, xyz, keep


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
    return theta, rot, trans


def yaw_from_quat(qx, qy, qz, qw):
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def quat_from_yaw(yaw):
    return 0.0, 0.0, math.sin(0.5 * yaw), math.cos(0.5 * yaw)


def transform_rows(est, theta, rot, trans):
    out = est.copy()
    out[:, 1:3] = est[:, 1:3] @ rot.T + trans
    for i in range(len(out)):
        yaw = yaw_from_quat(out[i, 4], out[i, 5], out[i, 6], out[i, 7]) + theta
        out[i, 4:8] = quat_from_yaw(yaw)
    return out


def stats(values):
    return {
        "rmse_m": float(np.sqrt(np.mean(values * values))) if len(values) else float("nan"),
        "mean_m": float(np.mean(values)) if len(values) else float("nan"),
        "median_m": float(np.median(values)) if len(values) else float("nan"),
        "p95_m": float(np.percentile(values, 95)) if len(values) else float("nan"),
        "max_m": float(np.max(values)) if len(values) else float("nan"),
    }


def thin(n, max_points):
    if n <= max_points:
        return np.arange(n, dtype=int)
    return np.linspace(0, n - 1, max_points).astype(int)


def plot_overlay(out_path, title, plot_rows, metric_text):
    fig, ax = plt.subplots(figsize=(7.2, 7.2), dpi=180)
    all_xy = []
    for idx, item in enumerate(plot_rows):
        color = REPORT_COLORS[idx % len(REPORT_COLORS)]
        gt = item["gt_xy"]
        est = item["est_xy"]
        gt_idx = thin(len(gt), 1800)
        est_idx = thin(len(est), 1800)
        label = "%s / %s" % (item["robot"], item["gt"])
        ax.plot(gt[gt_idx, 0], gt[gt_idx, 1], color=color, linewidth=1.8, alpha=0.95, label="%s GT" % label)
        ax.plot(est[est_idx, 0], est[est_idx, 1], color=color, linewidth=1.35, alpha=0.85, linestyle="--", label="%s MESA" % label)
        all_xy.append(gt)
        all_xy.append(est)
    all_xy = np.vstack(all_xy)
    xmin, ymin = np.min(all_xy, axis=0)
    xmax, ymax = np.max(all_xy, axis=0)
    span = max(xmax - xmin, ymax - ymin)
    pad = 0.08 * span if span > 0 else 1.0
    cx = 0.5 * (xmin + xmax)
    cy = 0.5 * (ymin + ymax)
    half = 0.5 * span + pad
    ax.set_xlim(cx - half, cx + half)
    ax.set_ylim(cy - half, cy + half)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, color="#d9d9d9", linewidth=0.6, alpha=0.75)
    ax.set_title(title, fontsize=10)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.text(
        0.02,
        0.98,
        metric_text,
        transform=ax.transAxes,
        va="top",
        ha="left",
        fontsize=9,
        bbox={"facecolor": "white", "edgecolor": "0.75", "alpha": 0.88},
    )
    ax.legend(loc="lower right", fontsize=6.7, ncol=2, frameon=True, framealpha=0.92)
    fig.tight_layout(pad=0.8)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(str(out_path))
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description="Evaluate multi-robot TUM estimates with one joint SE(2) GT alignment.")
    parser.add_argument("--mapping", required=True, help="CSV with robot,gt,offset_s columns.")
    parser.add_argument("--estimate-template", required=True)
    parser.add_argument("--gt-template", required=True)
    parser.add_argument("--method", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--title", default="")
    parser.add_argument("--exclusion-windows", default="")
    parser.add_argument("--exclude-min-duration", type=float, default=1.0)
    parser.add_argument("--exclude-reasons", default="z_offset")
    args = parser.parse_args()

    out_dir = Path(args.output_dir)
    aligned_dir = out_dir / "aligned_tum" / args.method
    residual_path = out_dir / "residuals" / ("%s_residuals.csv" % args.method)
    summary_path = out_dir / "tables" / ("%s_summary.csv" % args.method)
    plot_path = out_dir / "plots" / ("%s_vs_mocap_same_map.png" % args.method)
    residual_path.parent.mkdir(parents=True, exist_ok=True)
    summary_path.parent.mkdir(parents=True, exist_ok=True)

    exclusion_windows = load_exclusion_windows(args.exclusion_windows, args.exclude_min_duration, args.exclude_reasons)

    records = []
    all_est_xy = []
    all_gt_xy = []
    for row in load_mapping(args.mapping):
        robot = row["robot"]
        gt_label = row["gt"]
        offset = float(row.get("offset_s", 0.0))
        est = read_tum(args.estimate_template.format(robot=robot, gt=gt_label))
        gt = read_tum(args.gt_template.format(robot=robot, gt=gt_label))
        gt_times, gt_xyz, keep = interp_xyz(gt, est[:, 0] + offset)
        est_kept = est[keep]
        window_keep = not_excluded(gt_times, exclusion_windows.get(gt_label, []))
        gt_times = gt_times[window_keep]
        gt_xyz = gt_xyz[window_keep]
        est_kept = est_kept[window_keep]
        if len(est_kept) < 3:
            raise RuntimeError("not enough matched pairs for %s -> %s" % (robot, gt_label))
        records.append(
            {
                "robot": robot,
                "gt": gt_label,
                "offset": offset,
                "est": est,
                "est_pairs": est_kept,
                "gt_times": gt_times,
                "gt_xyz": gt_xyz,
            }
        )
        all_est_xy.append(est_kept[:, 1:3])
        all_gt_xy.append(gt_xyz[:, :2])

    theta, rot, trans = fit_se2(np.vstack(all_est_xy), np.vstack(all_gt_xy))

    residual_rows = []
    summary_rows = []
    plot_rows = []
    for rec in records:
        aligned_all = transform_rows(rec["est"], theta, rot, trans)
        aligned_pairs = transform_rows(rec["est_pairs"], theta, rot, trans)
        err_xy = np.linalg.norm(aligned_pairs[:, 1:3] - rec["gt_xyz"][:, :2], axis=1)
        row_stats = stats(err_xy)
        row_stats.update(
            {
                "method": args.method,
                "robot": rec["robot"],
                "gt": rec["gt"],
                "offset_s": rec["offset"],
                "matched": len(err_xy),
                "theta_rad": theta,
                "tx_m": float(trans[0]),
                "ty_m": float(trans[1]),
            }
        )
        summary_rows.append(row_stats)
        aligned_name = "%s_%s_to_%s_joint_aligned.tum" % (args.method, rec["robot"], rec["gt"])
        write_tum(aligned_dir / aligned_name, aligned_all)
        for stamp_est, stamp_gt, est_row, gt_point, err in zip(
            rec["est_pairs"][:, 0], rec["gt_times"], aligned_pairs, rec["gt_xyz"], err_xy
        ):
            residual_rows.append(
                {
                    "method": args.method,
                    "robot": rec["robot"],
                    "gt": rec["gt"],
                    "stamp_est": "%.9f" % stamp_est,
                    "stamp_gt": "%.9f" % stamp_gt,
                    "est_x_aligned": "%.9f" % est_row[1],
                    "est_y_aligned": "%.9f" % est_row[2],
                    "gt_x": "%.9f" % gt_point[0],
                    "gt_y": "%.9f" % gt_point[1],
                    "err_xy_m": "%.9f" % err,
                }
            )
        plot_rows.append({"robot": rec["robot"], "gt": rec["gt"], "est_xy": aligned_pairs[:, 1:3], "gt_xy": rec["gt_xyz"][:, :2]})

    with summary_path.open("w", newline="") as f:
        fieldnames = [
            "method",
            "robot",
            "gt",
            "offset_s",
            "matched",
            "rmse_m",
            "mean_m",
            "median_m",
            "p95_m",
            "max_m",
            "theta_rad",
            "tx_m",
            "ty_m",
        ]
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(summary_rows)

    with residual_path.open("w", newline="") as f:
        fieldnames = ["method", "robot", "gt", "stamp_est", "stamp_gt", "est_x_aligned", "est_y_aligned", "gt_x", "gt_y", "err_xy_m"]
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(residual_rows)

    all_err = np.asarray([float(row["err_xy_m"]) for row in residual_rows], dtype=float)
    aggregate = stats(all_err)
    metric_text = "joint SE(2) alignment\n2D ATE RMSE %.3f m\nmedian %.3f m, p95 %.3f m" % (
        aggregate["rmse_m"],
        aggregate["median_m"],
        aggregate["p95_m"],
    )
    plot_overlay(plot_path, args.title or ("%s vs Mocap GT" % args.method), plot_rows, metric_text)

    print("wrote %s" % summary_path)
    print("wrote %s" % residual_path)
    print("wrote aligned trajectories under %s" % aligned_dir)
    print("wrote %s" % plot_path)


if __name__ == "__main__":
    main()
