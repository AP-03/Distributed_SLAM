#!/usr/bin/env python
from __future__ import print_function

import argparse
import csv
import itertools
import math
import os

import rosbag


def stamp_from_msg(topic_time, msg):
    if hasattr(msg, "header"):
        stamp = msg.header.stamp
        if stamp.secs != 0 or stamp.nsecs != 0:
            return stamp.to_sec()
    return topic_time.to_sec()


def pose_xy(msg):
    if hasattr(msg, "pose") and hasattr(msg.pose, "pose"):
        pose = msg.pose.pose
    elif hasattr(msg, "pose"):
        pose = msg.pose
    else:
        raise ValueError("expected Odometry or PoseStamped-like message")
    return pose.position.x, pose.position.y


def maybe_append(track, t, x, y, min_dt):
    if track and min_dt > 0 and t - track[-1][0] < min_dt:
        return
    track.append((t, x, y))


def read_track(bag_path, topic, max_rate):
    track = []
    min_dt = 1.0 / max_rate if max_rate and max_rate > 0 else 0.0
    with rosbag.Bag(bag_path, "r") as bag:
        for _, msg, topic_time in bag.read_messages(topics=[topic]):
            t = stamp_from_msg(topic_time, msg)
            x, y = pose_xy(msg)
            if all(v == v and not math.isinf(v) for v in [t, x, y]):
                maybe_append(track, t, x, y, min_dt)
    return track


def interpolate_xy(track, t):
    if not track or t < track[0][0] or t > track[-1][0]:
        return None
    lo = 0
    hi = len(track) - 1
    while lo <= hi:
        mid = (lo + hi) // 2
        if track[mid][0] < t:
            lo = mid + 1
        else:
            hi = mid - 1
    idx = max(1, lo)
    t0, x0, y0 = track[idx - 1]
    t1, x1, y1 = track[idx]
    if t1 <= t0:
        return x0, y0
    a = (t - t0) / (t1 - t0)
    return x0 + a * (x1 - x0), y0 + a * (y1 - y0)


def path_length(track):
    total = 0.0
    for prev, cur in zip(track, track[1:]):
        total += math.hypot(cur[1] - prev[1], cur[2] - prev[2])
    return total


def fit_se2_rmse(src_xy, dst_xy):
    n = len(src_xy)
    if n < 3:
        return None
    sx = sum(p[0] for p in src_xy) / n
    sy = sum(p[1] for p in src_xy) / n
    dx = sum(p[0] for p in dst_xy) / n
    dy = sum(p[1] for p in dst_xy) / n

    c = 0.0
    s = 0.0
    for (x, y), (u, v) in zip(src_xy, dst_xy):
        ax = x - sx
        ay = y - sy
        bx = u - dx
        by = v - dy
        c += ax * bx + ay * by
        s += ax * by - ay * bx

    theta = math.atan2(s, c)
    ct = math.cos(theta)
    st = math.sin(theta)
    tx = dx - (ct * sx - st * sy)
    ty = dy - (st * sx + ct * sy)

    err2 = []
    for (x, y), (u, v) in zip(src_xy, dst_xy):
        px = ct * x - st * y + tx
        py = st * x + ct * y + ty
        err2.append((px - u) * (px - u) + (py - v) * (py - v))
    return math.sqrt(sum(err2) / n), theta, tx, ty


def score_pair(odom, gt, offset_min, offset_max, offset_step, min_pairs):
    best = None
    steps = int(round((offset_max - offset_min) / offset_step))
    for i in range(steps + 1):
        offset = offset_min + i * offset_step
        src = []
        dst = []
        for t, x, y in odom:
            gt_xy = interpolate_xy(gt, t + offset)
            if gt_xy is None:
                continue
            src.append((x, y))
            dst.append(gt_xy)
        if len(src) < min_pairs:
            continue
        fit = fit_se2_rmse(src, dst)
        if fit is None:
            continue
        rmse, theta, tx, ty = fit
        candidate = {
            "rmse": rmse,
            "offset": offset,
            "pairs": len(src),
            "theta": theta,
            "tx": tx,
            "ty": ty,
        }
        if best is None or candidate["rmse"] < best["rmse"]:
            best = candidate
    return best


def parse_robot_arg(spec):
    if "=" not in spec:
        raise ValueError("--robot must be name=/path/to.bag")
    name, path = spec.split("=", 1)
    return name, path


def main():
    parser = argparse.ArgumentParser(
        description="Rank robot bag to mocap label matches by odom/GT trajectory shape and clock offset."
    )
    parser.add_argument("--mocap-bag", required=True)
    parser.add_argument("--gt-topic", action="append", required=True, help="label=/gt/robot_XX/pose")
    parser.add_argument("--robot", action="append", required=True, help="label=/path/to/robot.bag")
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--max-rate", type=float, default=1.0)
    parser.add_argument("--offset-min", type=float, default=-10.0)
    parser.add_argument("--offset-max", type=float, default=10.0)
    parser.add_argument("--offset-step", type=float, default=0.25)
    parser.add_argument("--min-pairs", type=int, default=60)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()

    gt_tracks = {}
    for spec in args.gt_topic:
        label, topic = spec.split("=", 1)
        gt_tracks[label] = read_track(args.mocap_bag, topic, args.max_rate)
        print("GT %-10s %5d poses, path %.2f m" % (label, len(gt_tracks[label]), path_length(gt_tracks[label])))

    robot_tracks = {}
    for spec in args.robot:
        label, path = parse_robot_arg(spec)
        robot_tracks[label] = read_track(path, args.odom_topic, args.max_rate)
        print("ODOM %-8s %5d poses, path %.2f m" % (label, len(robot_tracks[label]), path_length(robot_tracks[label])))

    rows = []
    for robot_label, odom in sorted(robot_tracks.items()):
        for gt_label, gt in sorted(gt_tracks.items()):
            best = score_pair(odom, gt, args.offset_min, args.offset_max, args.offset_step, args.min_pairs)
            if best is None:
                continue
            row = {
                "robot": robot_label,
                "gt": gt_label,
                "rmse_m": "%.6f" % best["rmse"],
                "offset_s": "%.3f" % best["offset"],
                "pairs": str(best["pairs"]),
                "theta_rad": "%.6f" % best["theta"],
                "tx_m": "%.6f" % best["tx"],
                "ty_m": "%.6f" % best["ty"],
            }
            rows.append(row)

    rows.sort(key=lambda r: float(r["rmse_m"]))
    output_dir = os.path.dirname(os.path.abspath(args.output))
    if output_dir and not os.path.isdir(output_dir):
        os.makedirs(output_dir)
    with open(args.output, "w") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=["robot", "gt", "rmse_m", "offset_s", "pairs", "theta_rad", "tx_m", "ty_m"],
        )
        writer.writeheader()
        for row in rows:
            writer.writerow(row)

    print("\nTop pair candidates:")
    for row in rows[:18]:
        print(
            "%-8s -> %-8s  rmse=%6.3f m  offset=%7.3f s  pairs=%s"
            % (row["robot"], row["gt"], float(row["rmse_m"]), float(row["offset_s"]), row["pairs"])
        )

    robots = sorted(robot_tracks.keys())
    gts = sorted(gt_tracks.keys())
    cost = {(r["robot"], r["gt"]): float(r["rmse_m"]) for r in rows}
    best_assignment = None
    for perm in itertools.permutations(gts, len(robots)):
        total = 0.0
        ok = True
        for robot, gt in zip(robots, perm):
            if (robot, gt) not in cost:
                ok = False
                break
            total += cost[(robot, gt)]
        if ok and (best_assignment is None or total < best_assignment[0]):
            best_assignment = (total, list(zip(robots, perm)))
    if best_assignment:
        print("\nBest one-to-one assignment by total RMSE:")
        for robot, gt in best_assignment[1]:
            row = next(r for r in rows if r["robot"] == robot and r["gt"] == gt)
            print(
                "%-8s -> %-8s  rmse=%6.3f m  offset=%7.3f s"
                % (robot, gt, float(row["rmse_m"]), float(row["offset_s"]))
            )
        print("total_rmse_sum=%.3f m" % best_assignment[0])
    print("\nwrote %s" % args.output)


if __name__ == "__main__":
    main()
