#!/usr/bin/env python3
import argparse
import csv
import math
import os
import sqlite3

import cbor2


def yaw_to_quat(theta):
    half = 0.5 * theta
    return 0.0, 0.0, math.sin(half), math.cos(half)


def read_keymap(path):
    by_jrl_key = {}
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            jrl_key = int(row["jrl_key"])
            by_jrl_key[jrl_key] = {
                "robot": row["robot"],
                "local_index": int(row["local_index"]),
                "g2o_key": int(row["g2o_key"]),
            }
    return by_jrl_key


def read_stamps(db_path):
    con = sqlite3.connect(db_path)
    try:
        return {int(row[0]): float(row[1]) for row in con.execute("select id, stamp from Node")}
    finally:
        con.close()


def parse_db_map(items, fallback_db):
    if not items:
        if not fallback_db:
            raise ValueError("Provide --db or one or more --db-map robot=/path/to/rtabmap.db entries")
        return {"*": read_stamps(fallback_db)}

    out = {}
    for item in items:
        if "=" not in item:
            raise ValueError("--db-map entries must be robot=/path/to/rtabmap.db")
        robot, path = item.split("=", 1)
        robot = robot.strip()
        if not robot:
            raise ValueError("--db-map robot id is empty")
        out[robot] = read_stamps(path)
    return out


def value_to_tum_pose(value):
    if value["type"] == "Pose2":
        qx, qy, qz, qw = yaw_to_quat(float(value["theta"]))
        return float(value["x"]), float(value["y"]), 0.0, qx, qy, qz, qw
    if value["type"] == "Pose3":
        t = value["translation"]
        q = value["rotation"]
        return float(t[0]), float(t[1]), float(t[2]), float(q[1]), float(q[2]), float(q[3]), float(q[0])
    raise ValueError("Unsupported value type %s" % value["type"])


def main():
    parser = argparse.ArgumentParser(description="Export MESA/JRL result CBOR to per-robot TUM trajectories.")
    parser.add_argument("--result", required=True, help="MESA final_results.jrr.cbor file.")
    parser.add_argument("--keymap", required=True, help="Keymap CSV emitted next to the JRL file.")
    parser.add_argument("--db", help="Single RTAB-Map database containing Node id/stamp rows.")
    parser.add_argument(
        "--db-map",
        action="append",
        help="Per-robot RTAB-Map DB mapping, e.g. a=/path/agv0/rtabmap.db. Repeat once per robot.",
    )
    parser.add_argument("--output-dir", required=True, help="Directory for per-robot TUM files.")
    parser.add_argument(
        "--include-shared",
        action="store_true",
        help="Include shared variables from other robots in each robot solution.",
    )
    args = parser.parse_args()

    with open(args.result, "rb") as f:
        result = cbor2.load(f)

    keymap = read_keymap(args.keymap)
    stamps_by_robot = parse_db_map(args.db_map, args.db)
    os.makedirs(args.output_dir, exist_ok=True)

    dataset = result.get("dataset_name", "dataset")
    method = result.get("method_name", "method")

    for robot, values in sorted(result["solutions"].items()):
        rows = []
        skipped = 0
        for value in values:
            jrl_key = int(value["key"])
            mapping = keymap.get(jrl_key)
            if mapping is None:
                skipped += 1
                continue
            if not args.include_shared and mapping["robot"] != robot:
                continue
            stamps = stamps_by_robot.get(mapping["robot"], stamps_by_robot.get("*"))
            if stamps is None:
                skipped += 1
                continue
            stamp = stamps.get(mapping["g2o_key"])
            if stamp is None:
                skipped += 1
                continue
            rows.append((stamp,) + value_to_tum_pose(value))

        rows.sort(key=lambda r: r[0])
        out_path = os.path.join(args.output_dir, "%s_%s_%s.tum" % (dataset, method, robot))
        with open(out_path, "w") as f:
            for row in rows:
                f.write("%.9f %.9f %.9f %.9f %.9f %.9f %.9f %.9f\n" % row)
        print("wrote %d poses to %s (%d skipped)" % (len(rows), out_path, skipped))


if __name__ == "__main__":
    main()
