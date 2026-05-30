#!/usr/bin/env python3
"""ORKAR-style AprilTag/odometry consistency diagnostic for ROS1 exports.

This mirrors the lightweight mapping check used in ORKAR's Swarm-SLAM helpers:
before any tag observation is allowed to become an inter-robot constraint, score
each observed tag ID against each candidate target robot using only robot
odometry and the tag range estimate.  It is intentionally diagnostic; it does
not create MESA factors and it does not read mocap ground truth.
"""

import argparse
import csv
import json
import math
from bisect import bisect_left
from collections import Counter, defaultdict
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
            t, x, y, _z, qx, qy, qz, qw = map(float, parts[:8])
            rows.append([t, x, y, yaw_from_quat(qx, qy, qz, qw)])
    if not rows:
        raise RuntimeError("empty trajectory: %s" % path)
    return np.asarray(rows, dtype=float)


def nearest_pose(traj, stamp):
    times = traj[:, 0]
    idx = bisect_left(times, stamp)
    candidates = []
    if idx < len(traj):
        candidates.append(traj[idx])
    if idx > 0:
        candidates.append(traj[idx - 1])
    if not candidates:
        return None, float("inf")
    best = min(candidates, key=lambda row: abs(row[0] - stamp))
    return best, abs(best[0] - stamp)


def fit_circle(xy):
    x = xy[:, 0]
    y = xy[:, 1]
    amat = np.column_stack([x, y, np.ones_like(x)])
    bvec = -(x * x + y * y)
    a, b, c = np.linalg.lstsq(amat, bvec, rcond=-1)[0]
    cx = -a / 2.0
    cy = -b / 2.0
    radius = math.sqrt(max(0.0, cx * cx + cy * cy - c))
    radial = np.sqrt((x - cx) ** 2 + (y - cy) ** 2)
    return {
        "center_x": float(cx),
        "center_y": float(cy),
        "radius_m": float(radius),
        "radial_rmse_m": float(np.sqrt(np.mean((radial - radius) ** 2))),
    }


def read_detections(path, min_distance, max_distance):
    rows = []
    with open(path) as handle:
        for row in csv.DictReader(handle):
            try:
                distance = float(row["distance_m"])
                stamp = float(row.get("stamp") or row.get("header_stamp_ns") or row.get("bag_stamp_ns"))
                viewer = row["viewer_robot"]
                tag_id = int(row["tag_id"])
            except (KeyError, TypeError, ValueError):
                continue
            if distance < min_distance or distance > max_distance:
                continue
            rows.append(
                {
                    "viewer": viewer,
                    "stamp": stamp,
                    "tag_id": tag_id,
                    "distance": distance,
                }
            )
    return rows


def score_tag_targets(detections, odom, circles, max_sync_s, min_used):
    by_tag = defaultdict(list)
    for row in detections:
        by_tag[row["tag_id"]].append(row)

    scores = []
    robots = sorted(odom)
    for tag_id, tag_rows in sorted(by_tag.items()):
        for target in robots:
            used = []
            skipped_self = 0
            skipped_sync = 0
            viewer_counts = Counter()
            for det in tag_rows:
                viewer = det["viewer"]
                if viewer == target:
                    skipped_self += 1
                    continue
                viewer_pose, viewer_dt = nearest_pose(odom[viewer], det["stamp"])
                target_pose, target_dt = nearest_pose(odom[target], det["stamp"])
                if max(viewer_dt, target_dt) > max_sync_s:
                    skipped_sync += 1
                    continue

                cv = circles[viewer]
                ct = circles[target]
                viewer_xy = np.asarray(
                    [viewer_pose[1] - cv["center_x"], viewer_pose[2] - cv["center_y"]],
                    dtype=float,
                )
                target_xy = np.asarray(
                    [target_pose[1] - ct["center_x"], target_pose[2] - ct["center_y"]],
                    dtype=float,
                )
                expected = float(np.linalg.norm(target_xy - viewer_xy))
                used.append(det["distance"] - expected)
                viewer_counts[viewer] += 1

            row = {
                "tag_id": tag_id,
                "target_robot": target,
                "total_detections": len(tag_rows),
                "used": len(used),
                "skipped_self": skipped_self,
                "skipped_sync": skipped_sync,
                "self_fraction": skipped_self / float(max(1, len(tag_rows))),
                "viewer_counts": dict(viewer_counts),
            }
            if len(used) >= min_used:
                signed = np.asarray(used, dtype=float)
                err = np.abs(signed)
                row.update(
                    {
                        "median_abs_m": float(np.median(err)),
                        "p90_abs_m": float(np.percentile(err, 90)),
                        "bias_m": float(np.median(signed)),
                    }
                )
            else:
                row.update({"median_abs_m": None, "p90_abs_m": None, "bias_m": None})
            scores.append(row)
    return scores


def write_scores(path, scores):
    fields = [
        "tag_id",
        "target_robot",
        "total_detections",
        "used",
        "skipped_self",
        "skipped_sync",
        "self_fraction",
        "viewer_counts",
        "median_abs_m",
        "p90_abs_m",
        "bias_m",
    ]
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(scores)


def summarize(scores, max_self_count, max_self_fraction):
    by_tag = defaultdict(list)
    for row in scores:
        if row["median_abs_m"] is not None:
            by_tag[row["tag_id"]].append(row)

    summary = {"best_by_tag": {}, "best_unique_by_tag": {}}
    for tag_id, rows in sorted(by_tag.items()):
        rows.sort(key=lambda row: (row["median_abs_m"], row["p90_abs_m"], -row["used"]))
        best = rows[0]
        second = rows[1] if len(rows) > 1 else None
        summary["best_by_tag"][str(tag_id)] = dict(best)
        summary["best_by_tag"][str(tag_id)]["second_target_robot"] = second["target_robot"] if second else None
        summary["best_by_tag"][str(tag_id)]["median_margin_to_second_m"] = (
            second["median_abs_m"] - best["median_abs_m"] if second else None
        )

        unique = [
            row
            for row in rows
            if row["skipped_self"] <= max_self_count and row["self_fraction"] <= max_self_fraction
        ]
        unique.sort(key=lambda row: (row["median_abs_m"], row["p90_abs_m"], row["skipped_self"], -row["used"]))
        if unique:
            best = unique[0]
            second = unique[1] if len(unique) > 1 else None
            summary["best_unique_by_tag"][str(tag_id)] = dict(best)
            summary["best_unique_by_tag"][str(tag_id)]["second_target_robot"] = (
                second["target_robot"] if second else None
            )
            summary["best_unique_by_tag"][str(tag_id)]["median_margin_to_second_m"] = (
                second["median_abs_m"] - best["median_abs_m"] if second else None
            )
        else:
            summary["best_unique_by_tag"][str(tag_id)] = {
                "target_robot": None,
                "status": "no_low-self_candidate",
            }
    return summary


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--detections-csv", required=True)
    parser.add_argument("--odom-template", required=True, help="Template with {robot}, e.g. runs/.../{robot}_odom.tum")
    parser.add_argument("--robots", nargs="+", required=True)
    parser.add_argument("--out-dir", required=True)
    parser.add_argument("--min-distance", type=float, default=0.15)
    parser.add_argument("--max-distance", type=float, default=5.0)
    parser.add_argument("--max-sync-s", type=float, default=0.05)
    parser.add_argument("--min-used", type=int, default=8)
    parser.add_argument("--max-self-count", type=int, default=10)
    parser.add_argument("--max-self-fraction", type=float, default=0.10)
    return parser.parse_args()


def main():
    args = parse_args()
    out_dir = Path(args.out_dir)
    odom = {robot: read_tum(args.odom_template.format(robot=robot)) for robot in args.robots}
    circles = {robot: fit_circle(traj[:, 1:3]) for robot, traj in odom.items()}
    detections = read_detections(args.detections_csv, args.min_distance, args.max_distance)
    scores = score_tag_targets(detections, odom, circles, args.max_sync_s, args.min_used)

    scores_path = out_dir / "tag_target_range_scores_ros1.csv"
    write_scores(scores_path, scores)
    summary = summarize(scores, args.max_self_count, args.max_self_fraction)
    summary.update(
        {
            "detections_csv": args.detections_csv,
            "detections_after_filter": len(detections),
            "robots": args.robots,
            "filters": {
                "min_distance": args.min_distance,
                "max_distance": args.max_distance,
                "max_sync_s": args.max_sync_s,
                "min_used": args.min_used,
                "max_self_count": args.max_self_count,
                "max_self_fraction": args.max_self_fraction,
            },
            "odom_circle_stats": circles,
            "scores_csv": str(scores_path),
        }
    )
    summary_path = out_dir / "tag_mapping_summary_ros1.json"
    out_dir.mkdir(parents=True, exist_ok=True)
    summary_path.write_text(json.dumps(summary, indent=2, allow_nan=False))

    print("detections_after_filter=%d" % len(detections))
    for tag_id, row in summary["best_by_tag"].items():
        print(
            "tag %s best=%s median=%.3f p90=%.3f self=%.2f"
            % (
                tag_id,
                row["target_robot"],
                row["median_abs_m"],
                row["p90_abs_m"],
                row["self_fraction"],
            )
        )
    print("wrote %s" % scores_path)
    print("wrote %s" % summary_path)


if __name__ == "__main__":
    main()
