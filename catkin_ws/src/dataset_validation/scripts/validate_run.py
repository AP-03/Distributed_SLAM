#!/usr/bin/env python
from __future__ import print_function

import argparse
import csv
import os
import sys

import rosbag


def _default_required_topics(robot_id):
    return [
        "/tf",
        "/tf_static",
        "/phasespace/rigids",
        "/gt/%s/pose" % robot_id,
        "/gt/%s/odom" % robot_id,
        "/%s/odom" % robot_id,
    ]


def _stamp_from_message(msg, bag_time):
    if hasattr(msg, "header") and hasattr(msg.header, "stamp"):
        stamp = msg.header.stamp.to_sec()
        if stamp != 0.0:
            return stamp, False
        return bag_time.to_sec(), True
    return bag_time.to_sec(), False


def _topic_summary(bag, required_topics):
    topic_info = bag.get_type_and_topic_info()[1]
    summaries = {}

    for topic_name, info in topic_info.items():
        summaries[topic_name] = {
            "type": info.msg_type,
            "count": info.message_count,
            "frequency": info.frequency or 0.0,
            "first_stamp": None,
            "last_stamp": None,
            "zero_header_stamps": 0,
            "non_monotonic": 0,
        }

    topics_to_scan = list(set(required_topics + summaries.keys()))
    last_stamp_by_topic = {}
    for topic_name, msg, bag_time in bag.read_messages(topics=topics_to_scan):
        if topic_name not in summaries:
            continue
        stamp, zero_header = _stamp_from_message(msg, bag_time)
        summary = summaries[topic_name]
        if summary["first_stamp"] is None:
            summary["first_stamp"] = stamp
        summary["last_stamp"] = stamp
        if zero_header:
            summary["zero_header_stamps"] += 1
        if topic_name in last_stamp_by_topic and stamp < last_stamp_by_topic[topic_name]:
            summary["non_monotonic"] += 1
        last_stamp_by_topic[topic_name] = stamp

    return summaries


def _write_topic_rates(path, summaries):
    with open(path, "w") as handle:
        writer = csv.writer(handle)
        writer.writerow(["topic", "type", "count", "frequency_hz", "first_stamp", "last_stamp", "zero_header_stamps", "non_monotonic"])
        for topic_name in sorted(summaries.keys()):
            summary = summaries[topic_name]
            writer.writerow([
                topic_name,
                summary["type"],
                summary["count"],
                "%.6f" % summary["frequency"],
                "" if summary["first_stamp"] is None else "%.9f" % summary["first_stamp"],
                "" if summary["last_stamp"] is None else "%.9f" % summary["last_stamp"],
                summary["zero_header_stamps"],
                summary["non_monotonic"],
            ])


def _write_report(path, bag_path, required_topics, summaries):
    missing = [topic for topic in required_topics if topic not in summaries]
    non_monotonic = [topic for topic, summary in summaries.items() if summary["non_monotonic"] > 0]
    zero_headers = [topic for topic, summary in summaries.items() if summary["zero_header_stamps"] > 0]

    passed = len(missing) == 0 and len(non_monotonic) == 0

    with open(path, "w") as handle:
        handle.write("# Dataset Run Validation\n\n")
        handle.write("- Bag: `%s`\n" % bag_path)
        handle.write("- Result: **%s**\n\n" % ("PASS" if passed else "FAIL"))

        handle.write("## Required Topics\n\n")
        for topic in required_topics:
            handle.write("- [%s] `%s`\n" % ("x" if topic in summaries else " ", topic))
        handle.write("\n")

        if missing:
            handle.write("## Missing Topics\n\n")
            for topic in missing:
                handle.write("- `%s`\n" % topic)
            handle.write("\n")

        if non_monotonic:
            handle.write("## Non-Monotonic Timestamps\n\n")
            for topic in non_monotonic:
                handle.write("- `%s`: %d events\n" % (topic, summaries[topic]["non_monotonic"]))
            handle.write("\n")

        if zero_headers:
            handle.write("## Zero Header Stamps\n\n")
            for topic in zero_headers:
                handle.write("- `%s`: %d messages\n" % (topic, summaries[topic]["zero_header_stamps"]))
            handle.write("\n")

        handle.write("## Topic Summary\n\n")
        handle.write("| Topic | Type | Count | Rate Hz |\n")
        handle.write("| --- | --- | ---: | ---: |\n")
        for topic_name in sorted(summaries.keys()):
            summary = summaries[topic_name]
            handle.write("| `%s` | `%s` | %d | %.3f |\n" % (
                topic_name,
                summary["type"],
                summary["count"],
                summary["frequency"],
            ))

    return passed, missing, non_monotonic


def main():
    parser = argparse.ArgumentParser(description="Validate a one-robot dataset run")
    parser.add_argument("--robot_id", default="robot_01")
    parser.add_argument("--run_dir", required=True)
    parser.add_argument("--bag", default=None)
    parser.add_argument("--required_topic", action="append", default=None)
    args = parser.parse_args()

    run_dir = os.path.expanduser(args.run_dir)
    bag_path = os.path.expanduser(args.bag) if args.bag else os.path.join(run_dir, "raw", "%s.bag" % args.robot_id)
    validation_dir = os.path.join(run_dir, "validation")

    if not os.path.isfile(bag_path):
        print("ERROR: bag does not exist: %s" % bag_path)
        return 2
    if not os.path.isdir(validation_dir):
        os.makedirs(validation_dir)

    required_topics = args.required_topic or _default_required_topics(args.robot_id)

    bag = rosbag.Bag(bag_path, "r")
    try:
        summaries = _topic_summary(bag, required_topics)
    finally:
        bag.close()

    _write_topic_rates(os.path.join(validation_dir, "topic_rates.csv"), summaries)
    passed, missing, non_monotonic = _write_report(
        os.path.join(validation_dir, "report.md"),
        bag_path,
        required_topics,
        summaries,
    )

    print("Validation report: %s" % os.path.join(validation_dir, "report.md"))
    print("Topic rates: %s" % os.path.join(validation_dir, "topic_rates.csv"))
    if missing:
        print("Missing required topics: %s" % ", ".join(missing))
    if non_monotonic:
        print("Non-monotonic timestamps: %s" % ", ".join(non_monotonic))
    print("Result: %s" % ("PASS" if passed else "FAIL"))
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
