#!/usr/bin/env python
from __future__ import print_function

import argparse
import csv
import math
import os

import numpy as np
import rosbag


def quat_to_rot(q):
    x, y, z, w = [float(v) for v in q]
    n = x * x + y * y + z * z + w * w
    if n <= 0.0:
        return np.eye(3)
    s = 2.0 / n
    xx, yy, zz = x * x * s, y * y * s, z * z * s
    xy, xz, yz = x * y * s, x * z * s, y * z * s
    wx, wy, wz = w * x * s, w * y * s, w * z * s
    return np.array(
        [
            [1.0 - (yy + zz), xy - wz, xz + wy],
            [xy + wz, 1.0 - (xx + zz), yz - wx],
            [xz - wy, yz + wx, 1.0 - (xx + yy)],
        ],
        dtype=float,
    )


def transform_matrix(translation, rotation):
    out = np.eye(4)
    out[:3, :3] = quat_to_rot(rotation)
    out[:3, 3] = np.asarray(translation, dtype=float)
    return out


def yaw_from_rot(rot):
    return math.atan2(rot[1, 0], rot[0, 0])


def stamp_to_sec(stamp):
    return float(stamp.secs) + float(stamp.nsecs) * 1e-9


def clean_frame(frame):
    return frame.strip("/")


def read_tf_edges(bag_path):
    out = {}
    with rosbag.Bag(bag_path) as bag:
        for _, msg, _ in bag.read_messages(topics=["/tf_static", "/tf"]):
            for tr in msg.transforms:
                parent = clean_frame(tr.header.frame_id)
                child = clean_frame(tr.child_frame_id)
                t = tr.transform.translation
                q = tr.transform.rotation
                out[(parent, child)] = transform_matrix(
                    (t.x, t.y, t.z),
                    (q.x, q.y, q.z, q.w),
                )
    return out


def compose_tf_path(edges, frames):
    out = np.eye(4)
    for parent, child in zip(frames[:-1], frames[1:]):
        key = (parent, child)
        if key not in edges:
            raise RuntimeError("missing TF edge %s -> %s" % key)
        out = np.dot(out, edges[key])
    return out


def read_base_to_camera_optical(bag_path):
    edges = read_tf_edges(bag_path)
    paths = [
        ("base_footprint", "camera_link", "camera_aligned_depth_to_color_frame", "camera_color_optical_frame"),
        ("base_footprint", "camera_link", "camera_color_frame", "camera_color_optical_frame"),
        ("base_footprint", "camera_link", "camera_color_optical_frame"),
    ]
    errors = []
    for path in paths:
        try:
            return compose_tf_path(edges, path)
        except RuntimeError as exc:
            errors.append(str(exc))
    raise RuntimeError("could not compose base_footprint -> camera_color_optical_frame in %s: %s" % (bag_path, "; ".join(errors)))


def iter_detections(detection_bag, viewer_robot, base_to_camera):
    with rosbag.Bag(detection_bag) as bag:
        for _, msg, topic_time in bag.read_messages(topics=["/tag_detections"]):
            stamp = stamp_to_sec(msg.header.stamp)
            if stamp == 0.0:
                stamp = topic_time.to_sec()
            for det in msg.detections:
                if not det.id:
                    continue
                pose = det.pose.pose.pose
                p = pose.position
                q = pose.orientation
                camera_to_tag = transform_matrix(
                    (p.x, p.y, p.z),
                    (q.x, q.y, q.z, q.w),
                )
                base_to_tag = np.dot(base_to_camera, camera_to_tag)
                tag_id = int(det.id[0])
                size = float(det.size[0]) if det.size else float("nan")
                yield {
                    "viewer_robot": viewer_robot,
                    "stamp": "%.9f" % stamp,
                    "tag_id": tag_id,
                    "tag_size_m": "%.6f" % size,
                    "camera_frame": clean_frame(det.pose.header.frame_id),
                    "cam_x": "%.9f" % p.x,
                    "cam_y": "%.9f" % p.y,
                    "cam_z": "%.9f" % p.z,
                    "cam_qx": "%.9f" % q.x,
                    "cam_qy": "%.9f" % q.y,
                    "cam_qz": "%.9f" % q.z,
                    "cam_qw": "%.9f" % q.w,
                    "base_x": "%.9f" % base_to_tag[0, 3],
                    "base_y": "%.9f" % base_to_tag[1, 3],
                    "base_z": "%.9f" % base_to_tag[2, 3],
                    "base_yaw": "%.9f" % yaw_from_rot(base_to_tag[:3, :3]),
                    "distance_m": "%.9f" % math.sqrt(p.x * p.x + p.y * p.y + p.z * p.z),
                }


def robot_ids(count):
    return ["robot_%02d" % i for i in range(1, count + 1)]


def main():
    parser = argparse.ArgumentParser(description="Export apriltag_ros detection bags to a flat CSV.")
    parser.add_argument("--scenario", required=True)
    parser.add_argument("--robots", type=int, default=6)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()

    rows = []
    for robot in robot_ids(args.robots):
        raw_bag = os.path.join(args.scenario, "raw", "robots", robot + ".bag")
        detection_bag = os.path.join(args.scenario, "processed", "apriltag", robot, "detections.bag")
        if not os.path.exists(raw_bag):
            raise RuntimeError("missing raw bag %s" % raw_bag)
        if not os.path.exists(detection_bag):
            raise RuntimeError("missing detection bag %s" % detection_bag)
        base_to_camera = read_base_to_camera_optical(raw_bag)
        rows.extend(iter_detections(detection_bag, robot, base_to_camera))

    rows.sort(key=lambda r: (r["viewer_robot"], float(r["stamp"]), int(r["tag_id"])))
    out_dir = os.path.dirname(args.output)
    if out_dir:
        if not os.path.isdir(out_dir):
            os.makedirs(out_dir)
    fields = [
        "viewer_robot",
        "stamp",
        "tag_id",
        "tag_size_m",
        "camera_frame",
        "cam_x",
        "cam_y",
        "cam_z",
        "cam_qx",
        "cam_qy",
        "cam_qz",
        "cam_qw",
        "base_x",
        "base_y",
        "base_z",
        "base_yaw",
        "distance_m",
    ]
    with open(args.output, "w") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)
    print("wrote %d detections to %s" % (len(rows), args.output))


if __name__ == "__main__":
    main()
