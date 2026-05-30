#!/usr/bin/env python
from __future__ import print_function

import argparse
import csv
import math
import os

import rosbag


def pose_from_msg(msg):
    if hasattr(msg, "pose") and hasattr(msg.pose, "pose"):
        return msg.pose.pose
    if hasattr(msg, "pose"):
        return msg.pose
    raise ValueError("expected Odometry or PoseStamped-like message")


def stamp_from_msg(topic_time, msg):
    if hasattr(msg, "header"):
        stamp = msg.header.stamp
        if stamp.secs != 0 or stamp.nsecs != 0:
            return stamp.to_sec()
    return topic_time.to_sec()


def finite(*vals):
    return all(v == v and not math.isinf(v) for v in vals)


def read_track(bag_path, topic, max_rate):
    min_dt = 1.0 / max_rate if max_rate and max_rate > 0 else 0.0
    track = []
    with rosbag.Bag(bag_path, "r") as bag:
        for _, msg, topic_time in bag.read_messages(topics=[topic]):
            t = stamp_from_msg(topic_time, msg)
            pose = pose_from_msg(msg)
            x = pose.position.x
            y = pose.position.y
            z = pose.position.z
            if not finite(t, x, y, z):
                continue
            if track and min_dt > 0 and t - track[-1][0] < min_dt:
                continue
            track.append((t, x, y, z))
    return track


def median(values):
    if not values:
        return float("nan")
    vals = sorted(values)
    n = len(vals)
    mid = n // 2
    if n % 2:
        return vals[mid]
    return 0.5 * (vals[mid - 1] + vals[mid])


def contiguous_windows(events, gap_s):
    windows = []
    current = None
    reasons = set()
    for event in events:
        start = float(event["start_s"])
        end = float(event["end_s"])
        reason = event["reason"]
        if current is None or start - current[1] > gap_s:
            if current is not None:
                windows.append((current[0], current[1], ",".join(sorted(reasons))))
            current = [start, end]
            reasons = set([reason])
        else:
            current[1] = max(current[1], end)
            reasons.add(reason)
    if current is not None:
        windows.append((current[0], current[1], ",".join(sorted(reasons))))
    return windows


def parse_topic_spec(spec):
    if "=" not in spec:
        raise ValueError("--topic must be label=/topic")
    return spec.split("=", 1)


def main():
    parser = argparse.ArgumentParser(description="Detect impossible mocap motion such as robot pickup/flying segments.")
    parser.add_argument("--bag", required=True)
    parser.add_argument("--topic", action="append", required=True, help="label=/gt/robot_XX/pose")
    parser.add_argument("--output", required=True, help="Output anomaly CSV.")
    parser.add_argument("--windows-output", default="", help="Optional merged exclusion-window CSV.")
    parser.add_argument("--max-rate", type=float, default=60.0)
    parser.add_argument("--max-xy-speed", type=float, default=1.2, help="Flag XY speed above this m/s.")
    parser.add_argument("--max-z-speed", type=float, default=0.25, help="Flag vertical speed above this m/s.")
    parser.add_argument("--max-step-3d", type=float, default=0.20, help="Flag per-sample 3D jump above this many meters.")
    parser.add_argument("--max-z-above-floor", type=float, default=0.25, help="Flag z above median z by this many meters.")
    parser.add_argument("--merge-gap", type=float, default=1.0)
    args = parser.parse_args()

    rows = []
    window_rows = []
    for spec in args.topic:
        label, topic = parse_topic_spec(spec)
        track = read_track(args.bag, topic, args.max_rate)
        if len(track) < 2:
            print("%-10s insufficient poses: %d" % (label, len(track)))
            continue
        z0 = median([p[3] for p in track])
        events = []
        xy_speeds = []
        z_speeds = []
        z_offsets = []
        for prev, cur in zip(track, track[1:]):
            t0, x0, y0, z_prev = prev
            t1, x1, y1, z_cur = cur
            dt = t1 - t0
            if dt <= 0:
                continue
            dxy = math.hypot(x1 - x0, y1 - y0)
            dz = abs(z_cur - z_prev)
            d3 = math.sqrt(dxy * dxy + dz * dz)
            xy_speed = dxy / dt
            z_speed = dz / dt
            z_above = abs(z_cur - z0)
            xy_speeds.append(xy_speed)
            z_speeds.append(z_speed)
            z_offsets.append(z_above)
            checks = [
                ("xy_speed", xy_speed, args.max_xy_speed),
                ("z_speed", z_speed, args.max_z_speed),
                ("step_3d", d3, args.max_step_3d),
                ("z_offset", z_above, args.max_z_above_floor),
            ]
            for reason, value, threshold in checks:
                if value > threshold:
                    event = {
                        "label": label,
                        "topic": topic,
                        "start_s": "%.9f" % t0,
                        "end_s": "%.9f" % t1,
                        "reason": reason,
                        "value": "%.9f" % value,
                        "threshold": "%.9f" % threshold,
                        "x": "%.9f" % x1,
                        "y": "%.9f" % y1,
                        "z": "%.9f" % z_cur,
                    }
                    rows.append(event)
                    events.append(event)
        windows = contiguous_windows(events, args.merge_gap)
        for start, end, reasons in windows:
            window_rows.append(
                {
                    "label": label,
                    "topic": topic,
                    "start_s": "%.9f" % start,
                    "end_s": "%.9f" % end,
                    "duration_s": "%.3f" % (end - start),
                    "reasons": reasons,
                }
            )
        print(
            "%-10s poses=%5d duration=%7.2fs z_median=% .3f "
            "max_xy_speed=%5.2f max_z_speed=%5.2f max_z_offset=%5.2f events=%d windows=%d"
            % (
                label,
                len(track),
                track[-1][0] - track[0][0],
                z0,
                max(xy_speeds) if xy_speeds else 0.0,
                max(z_speeds) if z_speeds else 0.0,
                max(z_offsets) if z_offsets else 0.0,
                len(events),
                len(windows),
            )
        )

    output_dir = os.path.dirname(os.path.abspath(args.output))
    if output_dir and not os.path.isdir(output_dir):
        os.makedirs(output_dir)
    with open(args.output, "w") as f:
        fieldnames = ["label", "topic", "start_s", "end_s", "reason", "value", "threshold", "x", "y", "z"]
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)

    if args.windows_output:
        windows_dir = os.path.dirname(os.path.abspath(args.windows_output))
        if windows_dir and not os.path.isdir(windows_dir):
            os.makedirs(windows_dir)
        with open(args.windows_output, "w") as f:
            fieldnames = ["label", "topic", "start_s", "end_s", "duration_s", "reasons"]
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            for row in window_rows:
                writer.writerow(row)
        print("wrote %s" % args.windows_output)
    print("wrote %s" % args.output)


if __name__ == "__main__":
    main()
