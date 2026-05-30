#!/usr/bin/env python
from __future__ import print_function

import argparse
import math

import rosbag


def pose_from_msg(msg):
    if hasattr(msg, "pose") and hasattr(msg.pose, "pose"):
        return msg.pose.pose
    if hasattr(msg, "pose"):
        return msg.pose
    raise ValueError("Unsupported message type: expected Odometry or PoseStamped-like message")


def stamp_from_msg(topic_time, msg, use_bag_time):
    if use_bag_time or not hasattr(msg, "header"):
        return topic_time.to_sec()
    stamp = msg.header.stamp
    if stamp.secs == 0 and stamp.nsecs == 0:
        return topic_time.to_sec()
    return stamp.to_sec()


def finite_pose(pose):
    vals = [
        pose.position.x,
        pose.position.y,
        pose.position.z,
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    ]
    return all(math.isfinite(v) if hasattr(math, "isfinite") else v == v for v in vals)


def main():
    parser = argparse.ArgumentParser(description="Export a ROS bag pose/odom topic to TUM trajectory format.")
    parser.add_argument("--bag", required=True, help="Input ROS bag.")
    parser.add_argument("--topic", required=True, help="PoseStamped, PoseWithCovarianceStamped, or Odometry topic.")
    parser.add_argument("--output", required=True, help="Output TUM text file.")
    parser.add_argument("--use-bag-time", action="store_true", help="Use bag receive time instead of message stamp.")
    parser.add_argument("--max-rate", type=float, default=0.0, help="Optional max export rate in Hz.")
    args = parser.parse_args()

    min_dt = 1.0 / args.max_rate if args.max_rate and args.max_rate > 0 else 0.0
    last_t = None
    rows = []

    with rosbag.Bag(args.bag) as bag:
        for _, msg, topic_time in bag.read_messages(topics=[args.topic]):
            t = stamp_from_msg(topic_time, msg, args.use_bag_time)
            if last_t is not None and min_dt > 0 and t - last_t < min_dt:
                continue
            pose = pose_from_msg(msg)
            if not finite_pose(pose):
                continue
            rows.append(
                (
                    t,
                    pose.position.x,
                    pose.position.y,
                    pose.position.z,
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w,
                )
            )
            last_t = t

    rows.sort(key=lambda r: r[0])
    with open(args.output, "w") as f:
        for row in rows:
            f.write("%.9f %.9f %.9f %.9f %.9f %.9f %.9f %.9f\n" % row)

    print("wrote %d poses to %s" % (len(rows), args.output))


if __name__ == "__main__":
    main()
