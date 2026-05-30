#!/usr/bin/env python3
import argparse
import os

import matplotlib.pyplot as plt


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
            if len(parts) < 8:
                continue
            rows.append(tuple(float(x) for x in parts[:8]))
    return rows


def main():
    parser = argparse.ArgumentParser(description="Plot XY trajectories from TUM files.")
    parser.add_argument("--traj", action="append", required=True, help="Trajectory as label=/path/to/file.tum")
    parser.add_argument("--output", required=True, help="Output plot path, e.g. plot.png")
    parser.add_argument("--title", default="", help="Optional plot title.")
    parser.add_argument("--metric-text", default="", help="Optional report metric box text.")
    parser.add_argument("--max-points", type=int, default=2500, help="Maximum points per trajectory in the plot.")
    parser.add_argument("--color-gt", action="store_true", help="Use palette colours for GT/mocap trajectories.")
    args = parser.parse_args()

    fig, ax = plt.subplots(figsize=(7.2, 7.2), dpi=180)
    all_x = []
    all_y = []
    for idx, spec in enumerate(args.traj):
        if "=" not in spec:
            raise ValueError("--traj must be label=/path/to/file.tum")
        label, path = spec.split("=", 1)
        rows = read_tum(path)
        if not rows:
            continue
        if len(rows) > args.max_points:
            step_idx = [int(round(i)) for i in [j * (len(rows) - 1) / float(args.max_points - 1) for j in range(args.max_points)]]
            rows = [rows[i] for i in step_idx]

        xs = [r[1] for r in rows]
        ys = [r[2] for r in rows]
        all_x.extend(xs)
        all_y.extend(ys)

        lower_label = label.lower()
        is_gt = "gt" in lower_label or "ground" in lower_label or "mocap" in lower_label
        is_odom = "odom" in lower_label
        color = "black" if is_gt and not args.color_gt else REPORT_COLORS[idx % len(REPORT_COLORS)]
        linestyle = "--" if is_gt or is_odom else "-"
        linewidth = 1.35 if is_gt else 1.55
        alpha = 0.82 if is_gt else 0.9
        ax.plot(xs, ys, label=label, linewidth=linewidth, color=color, linestyle=linestyle, alpha=alpha, solid_capstyle="round")

    if all_x and all_y:
        xmin, xmax = min(all_x), max(all_x)
        ymin, ymax = min(all_y), max(all_y)
        span = max(xmax - xmin, ymax - ymin)
        pad = 0.08 * span if span > 0 else 1.0
        cx = 0.5 * (xmin + xmax)
        cy = 0.5 * (ymin + ymax)
        half = 0.5 * span + pad
        ax.set_xlim(cx - half, cx + half)
        ax.set_ylim(cy - half, cy + half)

    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, color="#d9d9d9", linewidth=0.6, alpha=0.75)
    if args.title:
        ax.set_title(args.title, fontsize=10)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    if args.metric_text:
        ax.text(
            0.02,
            0.98,
            args.metric_text,
            transform=ax.transAxes,
            va="top",
            ha="left",
            fontsize=9,
            bbox={"facecolor": "white", "edgecolor": "0.75", "alpha": 0.88},
        )
    ax.legend(loc="upper right", fontsize=7.6, frameon=True, framealpha=0.92)
    fig.tight_layout(pad=0.8)
    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    fig.savefig(args.output)
    print("wrote %s" % args.output)


if __name__ == "__main__":
    main()
