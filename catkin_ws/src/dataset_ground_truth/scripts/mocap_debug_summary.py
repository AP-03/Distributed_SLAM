#!/usr/bin/env python
from __future__ import print_function

import argparse
import math
import os
from collections import defaultdict

import rosbag


def _stamp(msg, bag_time):
    if hasattr(msg, "header") and hasattr(msg.header, "stamp") and msg.header.stamp.to_sec() != 0.0:
        return msg.header.stamp.to_sec()
    return bag_time.to_sec()


def _new_stats():
    return {
        "observed": 0,
        "skipped": 0,
        "count": 0,
        "first": None,
        "last": None,
        "min": [float("inf"), float("inf"), float("inf")],
        "max": [float("-inf"), float("-inf"), float("-inf")],
        "cond_sum": 0.0,
        "cond_min": float("inf"),
        "last_point": None,
        "path_length": 0.0,
    }


def _valid_sample(x, y, z, cond, min_condition, include_invalid):
    if include_invalid:
        return True
    if cond <= min_condition:
        return False
    return abs(x) > 1e-9 or abs(y) > 1e-9 or abs(z) > 1e-9


def _update(stats, timestamp, x, y, z, cond):
    stats["count"] += 1
    stats["first"] = timestamp if stats["first"] is None else min(stats["first"], timestamp)
    stats["last"] = timestamp if stats["last"] is None else max(stats["last"], timestamp)
    stats["min"][0] = min(stats["min"][0], x)
    stats["min"][1] = min(stats["min"][1], y)
    stats["min"][2] = min(stats["min"][2], z)
    stats["max"][0] = max(stats["max"][0], x)
    stats["max"][1] = max(stats["max"][1], y)
    stats["max"][2] = max(stats["max"][2], z)
    stats["cond_sum"] += cond
    stats["cond_min"] = min(stats["cond_min"], cond)

    point = (x, y, z)
    if stats["last_point"] is not None:
        lx, ly, lz = stats["last_point"]
        stats["path_length"] += math.sqrt((x - lx) ** 2 + (y - ly) ** 2 + (z - lz) ** 2)
    stats["last_point"] = point


def _format_table(title, entries):
    lines = []
    lines.append("## %s" % title)
    lines.append("")
    if not entries:
        lines.append("No messages found.")
        lines.append("")
        return lines

    lines.append("| ID | Observed | Valid | Skipped | Duration s | Path m | Mean Cond | Min Cond | X Range | Y Range | Z Range |")
    lines.append("| ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- | --- | --- |")
    for identifier in sorted(entries.keys()):
        s = entries[identifier]
        duration = (s["last"] - s["first"]) if s["first"] is not None and s["last"] is not None else 0.0
        mean_cond = s["cond_sum"] / s["count"] if s["count"] else 0.0
        min_cond = s["cond_min"] if s["count"] else 0.0
        x_range = "n/a" if s["count"] == 0 else "%.3f..%.3f" % (s["min"][0], s["max"][0])
        y_range = "n/a" if s["count"] == 0 else "%.3f..%.3f" % (s["min"][1], s["max"][1])
        z_range = "n/a" if s["count"] == 0 else "%.3f..%.3f" % (s["min"][2], s["max"][2])
        lines.append(
            "| %d | %d | %d | %d | %.3f | %.3f | %.3f | %.3f | %s | %s | %s |"
            % (
                identifier,
                s["observed"],
                s["count"],
                s["skipped"],
                duration,
                s["path_length"],
                mean_cond,
                min_cond,
                x_range,
                y_range,
                z_range,
            )
        )
    lines.append("")
    return lines


def summarize_bag(bag_path, markers_topic, rigids_topic, min_condition, include_invalid):
    led_stats = defaultdict(_new_stats)
    rigid_stats = defaultdict(_new_stats)
    topic_counts = defaultdict(int)

    bag = rosbag.Bag(bag_path, "r")
    try:
        for topic, msg, t in bag.read_messages(topics=[markers_topic, rigids_topic]):
            topic_counts[topic] += 1
            timestamp = _stamp(msg, t)
            if topic == markers_topic:
                for marker in msg.markers:
                    stats = led_stats[int(marker.id)]
                    stats["observed"] += 1
                    if _valid_sample(marker.x, marker.y, marker.z, marker.cond, min_condition, include_invalid):
                        _update(stats, timestamp, marker.x, marker.y, marker.z, marker.cond)
                    else:
                        stats["skipped"] += 1
            elif topic == rigids_topic:
                for rigid in msg.rigids:
                    stats = rigid_stats[int(rigid.id)]
                    stats["observed"] += 1
                    if _valid_sample(rigid.x, rigid.y, rigid.z, rigid.cond, min_condition, include_invalid):
                        _update(stats, timestamp, rigid.x, rigid.y, rigid.z, rigid.cond)
                    else:
                        stats["skipped"] += 1
    finally:
        bag.close()

    return topic_counts, led_stats, rigid_stats


def main():
    parser = argparse.ArgumentParser(description="Summarize mocap marker/rigid contents in a rosbag")
    parser.add_argument("--bag", required=True)
    parser.add_argument("--markers_topic", default="/phasespace/markers")
    parser.add_argument("--rigids_topic", default="/phasespace/rigids")
    parser.add_argument("--min_condition", type=float, default=0.0)
    parser.add_argument("--include_invalid", action="store_true")
    parser.add_argument("--output", default=None, help="Optional markdown report path")
    args = parser.parse_args()

    bag_path = os.path.expanduser(args.bag)
    topic_counts, led_stats, rigid_stats = summarize_bag(
        bag_path,
        args.markers_topic,
        args.rigids_topic,
        args.min_condition,
        args.include_invalid,
    )

    lines = []
    lines.append("# Mocap Debug Summary")
    lines.append("")
    lines.append("- Bag: `%s`" % bag_path)
    lines.append("- `%s`: %d messages" % (args.markers_topic, topic_counts[args.markers_topic]))
    lines.append("- `%s`: %d messages" % (args.rigids_topic, topic_counts[args.rigids_topic]))
    lines.append("- Valid sample rule: `cond > %.3f` and non-zero position" % args.min_condition)
    lines.append("")
    lines.extend(_format_table("LED Marker Paths", led_stats))
    lines.extend(_format_table("Rigid Body Paths", rigid_stats))

    report = "\n".join(lines)
    print(report)

    if args.output:
        output_path = os.path.expanduser(args.output)
        output_dir = os.path.dirname(output_path)
        if output_dir and not os.path.isdir(output_dir):
            os.makedirs(output_dir)
        with open(output_path, "w") as handle:
            handle.write(report)
            handle.write("\n")


if __name__ == "__main__":
    main()
