#!/usr/bin/env python3
import argparse
import csv
import math
import os
import sqlite3
from bisect import bisect_left
from collections import defaultdict


def yaw_from_quat(qx, qy, qz, qw):
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def read_node_maps(db_path):
    con = sqlite3.connect(db_path)
    try:
        rows = con.execute("select id, map_id, stamp from Node order by id").fetchall()
    finally:
        con.close()
    node_map = {int(node_id): int(map_id) for node_id, map_id, _ in rows}
    stamps = {int(node_id): float(stamp) for node_id, _, stamp in rows}
    return node_map, stamps


def read_source_stamps(db_path):
    con = sqlite3.connect(db_path)
    try:
        rows = [(float(stamp), int(node_id)) for node_id, stamp in con.execute("select id, stamp from Node")]
    finally:
        con.close()
    rows.sort()
    return rows


def parse_db_map(items):
    out = {}
    for item in items or []:
        if "=" not in item:
            raise ValueError("--source-db-map entries must be robot=/path/to/rtabmap.db")
        robot, path = item.split("=", 1)
        robot = robot.strip()
        if len(robot) != 1:
            raise ValueError("--source-db-map robot ids must be single-character MESA ids")
        out[robot] = read_source_stamps(path)
    return out


def parse_robot_ids(value):
    robots = [item.strip() for item in value.split(",") if item.strip()]
    if not robots:
        raise ValueError("empty --robot-ids")
    if any(len(item) != 1 for item in robots):
        raise ValueError("--robot-ids entries must be single-character MESA robot ids, e.g. a,b,c")
    return robots


def infer_node_sources(output_stamps, source_stamps, max_stamp_diff):
    node_source = {}
    node_source_id = {}
    ambiguous = 0
    unmatched = 0
    for node_id, stamp in output_stamps.items():
        matches = []
        for robot, rows in source_stamps.items():
            stamps = [r[0] for r in rows]
            pos = bisect_left(stamps, stamp)
            candidates = []
            if pos < len(rows):
                candidates.append(rows[pos])
            if pos > 0:
                candidates.append(rows[pos - 1])
            if not candidates:
                continue
            best_stamp, best_id = min(candidates, key=lambda item: abs(item[0] - stamp))
            diff = abs(best_stamp - stamp)
            if diff <= max_stamp_diff:
                matches.append((diff, robot, best_id))
        if not matches:
            unmatched += 1
            continue
        matches.sort()
        if len(matches) > 1 and abs(matches[0][0] - matches[1][0]) < 1e-9:
            ambiguous += 1
        _, robot, source_id = matches[0]
        node_source[node_id] = robot
        node_source_id[node_id] = source_id
    return node_source, node_source_id, unmatched, ambiguous


def split_g2o(g2o_path, node_source):
    params = []
    vertices = defaultdict(list)
    intra_edges = defaultdict(list)
    inter_edges = []
    skipped = defaultdict(int)

    with open(g2o_path) as f:
        for line in f:
            parts = line.strip().split()
            if not parts:
                continue
            tag = parts[0]
            if tag.startswith("PARAMS"):
                params.append(line)
                continue
            if tag == "VERTEX_SE3:QUAT":
                node_id = int(parts[1])
                robot = node_source.get(node_id)
                if robot is None:
                    skipped["vertex_unknown_source"] += 1
                    continue
                vertices[robot].append(line)
                continue
            if tag == "EDGE_SE3:QUAT":
                from_id = int(parts[1])
                to_id = int(parts[2])
                from_robot = node_source.get(from_id)
                to_robot = node_source.get(to_id)
                if from_robot is None or to_robot is None:
                    skipped["edge_unknown_source"] += 1
                    continue
                if from_robot == to_robot:
                    intra_edges[from_robot].append(line)
                else:
                    x = float(parts[3])
                    y = float(parts[4])
                    qx = float(parts[6])
                    qy = float(parts[7])
                    qz = float(parts[8])
                    qw = float(parts[9])
                    theta = yaw_from_quat(qx, qy, qz, qw)
                    inter_edges.append((from_robot, from_robot, from_id, to_robot, to_id, x, y, theta))
                continue
    return params, vertices, intra_edges, inter_edges, skipped


def main():
    parser = argparse.ArgumentParser(
        description="Split a multi-session RTAB-Map g2o into per-robot g2o files and MESA inter-robot CSV."
    )
    parser.add_argument("--db", required=True, help="RTAB-Map multi-session database.")
    parser.add_argument("--g2o", required=True, help="RTAB-Map exported multi-session g2o graph.")
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--prefix", required=True)
    parser.add_argument("--robot-ids", default="a,b,c,d,e,f")
    parser.add_argument(
        "--source-db-map",
        action="append",
        help="Original robot DB mapping, e.g. a=/path/robot_01/rtabmap.db. Repeat once per robot.",
    )
    parser.add_argument("--max-stamp-diff", type=float, default=1e-6)
    parser.add_argument("--inter-csv", required=True)
    parser.add_argument("--inter-sigma-xy", type=float, default=0.20)
    parser.add_argument("--inter-sigma-theta", type=float, default=0.20)
    args = parser.parse_args()

    robots = parse_robot_ids(args.robot_ids)
    node_map, stamps = read_node_maps(args.db)
    source_stamps = parse_db_map(args.source_db_map)
    if not source_stamps:
        raise RuntimeError("provide --source-db-map entries so multi-session nodes can be assigned to source robots")
    node_source, node_source_id, unmatched, ambiguous = infer_node_sources(stamps, source_stamps, args.max_stamp_diff)
    if unmatched:
        raise RuntimeError("%d output nodes could not be assigned to a source robot by timestamp" % unmatched)

    params, vertices, intra_edges, inter_edges, skipped = split_g2o(args.g2o, node_source)
    os.makedirs(args.output_dir, exist_ok=True)

    graph_paths = {}
    active_robots = [robot for robot in robots if robot in set(node_source.values())]
    for robot in active_robots:
        path = os.path.join(args.output_dir, "%s_%s.g2o" % (args.prefix, robot))
        graph_paths[robot] = path
        with open(path, "w") as f:
            for line in params:
                f.write(line)
            for line in vertices[robot]:
                f.write(line)
            for line in intra_edges[robot]:
                f.write(line)

    with open(args.inter_csv, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                "owner_robot",
                "from_robot",
                "from_g2o_key",
                "to_robot",
                "to_g2o_key",
                "x",
                "y",
                "theta",
                "sigma_x",
                "sigma_y",
                "sigma_theta",
            ]
        )
        for owner, from_robot, from_id, to_robot, to_id, x, y, theta in inter_edges:
            writer.writerow(
                [
                    owner,
                    from_robot,
                    from_id,
                    to_robot,
                    to_id,
                    "%.9f" % x,
                    "%.9f" % y,
                    "%.9f" % theta,
                    "%.6f" % args.inter_sigma_xy,
                    "%.6f" % args.inter_sigma_xy,
                    "%.6f" % args.inter_sigma_theta,
                ]
            )

    map_csv = os.path.join(args.output_dir, "%s_map_sessions.csv" % args.prefix)
    with open(map_csv, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                "robot",
                "node_count",
                "min_output_node_id",
                "max_output_node_id",
                "min_source_node_id",
                "max_source_node_id",
                "min_stamp",
                "max_stamp",
                "map_ids",
                "graph",
            ]
        )
        for robot in active_robots:
            ids = [node_id for node_id, source_robot in node_source.items() if source_robot == robot]
            source_ids = [node_source_id[node_id] for node_id in ids]
            map_ids = sorted(set(node_map[node_id] for node_id in ids))
            writer.writerow(
                [
                    robot,
                    len(ids),
                    min(ids),
                    max(ids),
                    min(source_ids),
                    max(source_ids),
                    "%.9f" % min(stamps[i] for i in ids),
                    "%.9f" % max(stamps[i] for i in ids),
                    ";".join(str(item) for item in map_ids),
                    graph_paths[robot],
                ]
            )

    pair_counts = defaultdict(int)
    for _, from_robot, _, to_robot, _, _, _, _ in inter_edges:
        pair_counts[(from_robot, to_robot)] += 1

    print("wrote %d robot graphs to %s" % (len(active_robots), args.output_dir))
    print("wrote %d inter-robot edges to %s" % (len(inter_edges), args.inter_csv))
    print("source robots:", ", ".join(active_robots))
    if ambiguous:
        print("ambiguous timestamp assignments resolved by first nearest match:", ambiguous)
    if pair_counts:
        print("inter pairs:", ", ".join("%s-%s:%d" % (a, b, c) for (a, b), c in sorted(pair_counts.items())))
    if skipped:
        print("skipped:", dict(skipped))


if __name__ == "__main__":
    main()
