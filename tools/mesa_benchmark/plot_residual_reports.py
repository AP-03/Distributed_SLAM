#!/usr/bin/env python3
import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


COLORS = {
    "odom_tentative": "#1f77b4",
    "rtabmap_tentative": "#ff7f0e",
    "centralized": "#2ca02c",
    "geodesic_mesa": "#d62728",
}


LABELS = {
    "odom_tentative": "Wheel odometry",
    "rtabmap_tentative": "RTAB-Map frontend",
    "centralized": "Centralized backend",
    "geodesic_mesa": "MESA",
}


def read_residuals(path):
    rows = []
    with open(path) as f:
        for row in csv.DictReader(f):
            rows.append(
                {
                    "robot": row["robot"],
                    "stamp": float(row["stamp_gt"]),
                    "err": float(row["err_xy_m"]),
                }
            )
    return rows


def thin(values, max_points):
    if len(values) <= max_points:
        return np.arange(len(values), dtype=int)
    return np.linspace(0, len(values) - 1, max_points).astype(int)


def main():
    parser = argparse.ArgumentParser(description="Create report-style CDF and residual-over-time plots.")
    parser.add_argument("--residual", action="append", required=True, help="method=/path/to/residuals.csv")
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--prefix", default="residuals")
    args = parser.parse_args()

    out_dir = Path(args.output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    data = {}
    for spec in args.residual:
        method, path = spec.split("=", 1)
        data[method] = read_residuals(path)

    fig, ax = plt.subplots(figsize=(7.2, 4.8), dpi=180)
    for method, rows in data.items():
        err = np.asarray([row["err"] for row in rows], dtype=float)
        err = np.sort(err)
        y = np.linspace(0.0, 1.0, len(err), endpoint=True)
        ax.plot(err, y, linewidth=2.0, color=COLORS.get(method), label=LABELS.get(method, method))
    ax.set_title("S2B ATE residual CDF", fontsize=10)
    ax.set_xlabel("2D ATE [m]")
    ax.set_ylabel("fraction of matched poses")
    ax.grid(True, color="#d9d9d9", linewidth=0.6, alpha=0.75)
    ax.legend(loc="lower right", framealpha=0.92)
    fig.tight_layout(pad=0.8)
    fig.savefig(str(out_dir / ("%s_cdf.png" % args.prefix)))
    plt.close(fig)

    robots = sorted({row["robot"] for rows in data.values() for row in rows})
    fig, axes = plt.subplots(len(robots), 1, figsize=(8.4, max(5.0, 1.55 * len(robots))), dpi=180, sharex=True)
    if len(robots) == 1:
        axes = [axes]
    for ax, robot in zip(axes, robots):
        for method, rows in data.items():
            robot_rows = [row for row in rows if row["robot"] == robot]
            if not robot_rows:
                continue
            stamp = np.asarray([row["stamp"] for row in robot_rows], dtype=float)
            err = np.asarray([row["err"] for row in robot_rows], dtype=float)
            order = np.argsort(stamp)
            stamp = stamp[order]
            err = err[order]
            rel_t = stamp - stamp[0]
            idx = thin(rel_t, 1400)
            ax.plot(rel_t[idx], err[idx], linewidth=1.1, alpha=0.9, color=COLORS.get(method), label=LABELS.get(method, method))
        ax.set_ylabel("%s\nATE [m]" % robot)
        ax.grid(True, color="#d9d9d9", linewidth=0.6, alpha=0.75)
    axes[0].set_title("S2B ATE over time", fontsize=10)
    axes[-1].set_xlabel("time from first matched GT pose [s]")
    axes[0].legend(loc="upper right", fontsize=8, framealpha=0.92)
    fig.tight_layout(pad=0.8)
    fig.savefig(str(out_dir / ("%s_over_time.png" % args.prefix)))
    plt.close(fig)

    print("wrote %s" % (out_dir / ("%s_cdf.png" % args.prefix)))
    print("wrote %s" % (out_dir / ("%s_over_time.png" % args.prefix)))


if __name__ == "__main__":
    main()
