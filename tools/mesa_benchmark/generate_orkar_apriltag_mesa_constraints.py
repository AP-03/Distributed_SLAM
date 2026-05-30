#!/usr/bin/env python3
"""Generate MESA inter-robot edges from AprilTags using ORKAR-style gating.

Inputs are ROS1 AprilTag detections already exported in the viewer base frame,
per-robot odometry TUM files, RTAB-Map databases, and per-robot g2o graphs.

This does not use mocap.  Each tag rule supplies the robot carrying that tag and
the target-clock shift found by an odometry/tag consistency sweep:

    --tag-rule 14:robot_02:-90

meaning observations of tag 14 are treated as observations of robot_02 at
`viewer_stamp - 90`.
"""

import argparse
import csv
import json
import math
import os
import sqlite3
from bisect import bisect_left
from collections import Counter, defaultdict

import numpy as np
from scipy.optimize import least_squares


ROBOT_SYMBOLS = {
    "robot_01": "a",
    "robot_02": "b",
    "robot_03": "c",
    "robot_04": "d",
    "robot_05": "e",
    "robot_06": "f",
}


def wrap_angle(x):
    return (x + math.pi) % (2.0 * math.pi) - math.pi


def yaw_from_quat(qx, qy, qz, qw):
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def rot2(theta):
    c = math.cos(theta)
    s = math.sin(theta)
    return np.asarray([[c, -s], [s, c]], dtype=float)


def transform_point(pose, xy):
    return np.asarray([pose[0], pose[1]], dtype=float) + rot2(pose[2]).dot(np.asarray(xy, dtype=float))


def compose_pose(a, b):
    xy = transform_point(a, [b[0], b[1]])
    return np.asarray([xy[0], xy[1], wrap_angle(a[2] + b[2])], dtype=float)


def inverse_pose(a):
    yaw = wrap_angle(-a[2])
    xy = -rot2(yaw).dot(np.asarray([a[0], a[1]], dtype=float))
    return np.asarray([xy[0], xy[1], yaw], dtype=float)


def relative_pose(a, b):
    return compose_pose(inverse_pose(a), b)


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


def interp_pose(traj, stamp, max_dt):
    times = traj[:, 0]
    idx = bisect_left(times, stamp)
    if idx <= 0:
        if abs(times[0] - stamp) <= max_dt:
            return traj[0, 1:4].copy()
        return None
    if idx >= len(traj):
        if abs(times[-1] - stamp) <= max_dt:
            return traj[-1, 1:4].copy()
        return None
    t0 = times[idx - 1]
    t1 = times[idx]
    if min(abs(stamp - t0), abs(stamp - t1)) > max_dt and (t1 - t0) > max_dt:
        return None
    alpha = 0.0 if t1 == t0 else (stamp - t0) / (t1 - t0)
    xy = (1.0 - alpha) * traj[idx - 1, 1:3] + alpha * traj[idx, 1:3]
    yaw = traj[idx - 1, 3] + alpha * wrap_angle(traj[idx, 3] - traj[idx - 1, 3])
    return np.asarray([xy[0], xy[1], wrap_angle(yaw)], dtype=float)


def nearest_node(nodes, stamp, max_dt):
    times = [row[0] for row in nodes]
    idx = bisect_left(times, stamp)
    candidates = []
    if idx < len(nodes):
        candidates.append(nodes[idx])
    if idx > 0:
        candidates.append(nodes[idx - 1])
    if not candidates:
        return None
    best = min(candidates, key=lambda row: abs(row[0] - stamp))
    if abs(best[0] - stamp) > max_dt:
        return None
    return best


def read_graph_nodes(db_path, g2o_path):
    con = sqlite3.connect(db_path)
    try:
        stamps = {int(row[0]): float(row[1]) for row in con.execute("select id, stamp from Node")}
    finally:
        con.close()
    vertex_ids = set()
    with open(g2o_path) as handle:
        for line in handle:
            parts = line.split()
            if parts and parts[0] == "VERTEX_SE3:QUAT":
                vertex_ids.add(int(parts[1]))
    out = []
    for key in vertex_ids:
        if key in stamps:
            out.append((stamps[key], key))
    out.sort()
    return out


def parse_tag_rules(items):
    rules = {}
    for item in items:
        parts = item.split(":")
        if len(parts) != 3:
            raise ValueError("--tag-rule must be tag_id:target_robot:target_time_shift_s")
        tag = int(parts[0])
        rules[tag] = {"target": parts[1], "shift": float(parts[2])}
    return rules


def read_detections(path, rules, robots, min_distance, max_distance):
    rows = []
    robot_set = set(robots)
    with open(path) as handle:
        for row in csv.DictReader(handle):
            try:
                viewer = row["viewer_robot"]
                tag = int(row["tag_id"])
                stamp = float(row["stamp"])
                distance = float(row["distance_m"])
                z = np.asarray([float(row["base_x"]), float(row["base_y"]), float(row["base_yaw"])], dtype=float)
            except (KeyError, TypeError, ValueError):
                continue
            if tag not in rules or viewer not in robot_set:
                continue
            target = rules[tag]["target"]
            if target not in robot_set or target == viewer:
                continue
            if distance < min_distance or distance > max_distance:
                continue
            rows.append(
                {
                    "viewer": viewer,
                    "target": target,
                    "tag": tag,
                    "stamp": stamp,
                    "target_stamp": stamp + rules[tag]["shift"],
                    "shift": rules[tag]["shift"],
                    "z": z,
                    "distance": distance,
                }
            )
    rows.sort(key=lambda row: row["stamp"])
    return rows


def decimate_rows(rows, min_separation, max_per_viewer_tag):
    if min_separation <= 0 and max_per_viewer_tag <= 0:
        return rows
    out = []
    last = {}
    counts = Counter()
    for row in sorted(rows, key=lambda item: item["stamp"]):
        key = (row["viewer"], row["tag"])
        if max_per_viewer_tag > 0 and counts[key] >= max_per_viewer_tag:
            continue
        if min_separation > 0 and key in last and row["stamp"] - last[key] < min_separation:
            continue
        out.append(row)
        counts[key] += 1
        last[key] = row["stamp"]
    return out


def pack_params(robots, tags, align, offsets):
    vals = []
    for robot in robots[1:]:
        vals.extend(align[robot])
    for tag in tags:
        vals.extend(offsets[tag])
    return np.asarray(vals, dtype=float)


def unpack_params(params, robots, tags):
    pos = 0
    align = {robots[0]: np.asarray([0.0, 0.0, 0.0], dtype=float)}
    for robot in robots[1:]:
        align[robot] = np.asarray([params[pos], params[pos + 1], wrap_angle(params[pos + 2])], dtype=float)
        pos += 3
    offsets = {}
    for tag in tags:
        offsets[tag] = np.asarray([params[pos], params[pos + 1]], dtype=float)
        pos += 2
    return align, offsets


def fit_calibration(rows, odom, robots, args):
    tags = sorted({row["tag"] for row in rows})
    obs = []
    skipped = Counter()
    for row in rows:
        vp = interp_pose(odom[row["viewer"]], row["stamp"], args.max_odom_dt)
        tp = interp_pose(odom[row["target"]], row["target_stamp"], args.max_odom_dt)
        if vp is None or tp is None:
            skipped["missing_odom"] += 1
            continue
        obs.append((row, vp, tp))
    tag_counts = Counter(row["tag"] for row, _vp, _tp in obs)
    if not obs:
        raise RuntimeError("no observations after odom sync")

    def residuals(params):
        align, offsets = unpack_params(params, robots, tags)
        res = []
        for row, vp, tp in obs:
            tag_local_viewer_odom = transform_point(vp, row["z"][:2])
            tag_global_obs = transform_point(align[row["viewer"]], tag_local_viewer_odom)
            tag_local_target_odom = transform_point(tp, offsets[row["tag"]])
            tag_global_expected = transform_point(align[row["target"]], tag_local_target_odom)
            res.extend((tag_global_obs - tag_global_expected) / args.fit_sigma_xy)
        for tag in tags:
            offset_norm = float(np.linalg.norm(unpack_params(params, robots, tags)[1][tag]))
            res.append(max(0.0, offset_norm - args.max_tag_offset) / args.offset_excess_sigma)
        return np.asarray(res, dtype=float)

    align0 = {robot: np.asarray([0.0, 0.0, 0.0], dtype=float) for robot in robots}
    offsets0 = {tag: np.asarray([0.0, 0.0], dtype=float) for tag in tags}
    x0 = pack_params(robots, tags, align0, offsets0)
    opt = least_squares(residuals, x0, loss="soft_l1", f_scale=1.0, max_nfev=args.max_nfev)
    align, offsets = unpack_params(opt.x, robots, tags)

    residual_rows = []
    for row, vp, tp in obs:
        tag_local_viewer_odom = transform_point(vp, row["z"][:2])
        tag_global_obs = transform_point(align[row["viewer"]], tag_local_viewer_odom)
        tag_local_target_odom = transform_point(tp, offsets[row["tag"]])
        tag_global_expected = transform_point(align[row["target"]], tag_local_target_odom)
        err = tag_global_obs - tag_global_expected
        residual_rows.append((row, vp, tp, float(np.linalg.norm(err))))
    errors = np.asarray([x[3] for x in residual_rows], dtype=float)
    summary = {
        "observations": len(obs),
        "skipped": dict(skipped),
        "tags": tags,
        "tag_counts": dict(tag_counts),
        "median_residual_m": float(np.median(errors)),
        "p90_residual_m": float(np.percentile(errors, 90)),
        "rmse_residual_m": float(np.sqrt(np.mean(errors * errors))),
        "align": {robot: align[robot].tolist() for robot in robots},
        "offsets": {str(tag): offsets[tag].tolist() for tag in tags},
        "offset_norms": {str(tag): float(np.linalg.norm(offsets[tag])) for tag in tags},
    }
    return align, offsets, residual_rows, summary


def generate_constraints(residual_rows, align, offsets, graph_nodes, args):
    accepted = []
    audit = []
    pair_counts = Counter()
    last_pair_tag = {}
    for row, vp, tp, residual in sorted(residual_rows, key=lambda item: item[0]["stamp"]):
        reason = ""
        if residual > args.max_residual:
            reason = "residual"
        else:
            v_node = nearest_node(graph_nodes[row["viewer"]], row["stamp"], args.max_node_dt)
            t_node = nearest_node(graph_nodes[row["target"]], row["target_stamp"], args.max_node_dt)
            if v_node is None or t_node is None:
                reason = "missing_graph_node"
        broad_pair = (row["viewer"], row["target"])
        pair_tag = (row["viewer"], row["target"], row["tag"])
        if not reason and pair_counts[broad_pair] >= args.max_per_pair:
            reason = "pair_cap"
        if not reason and pair_tag in last_pair_tag and row["stamp"] - last_pair_tag[pair_tag] < args.min_separation:
            reason = "separation"

        if reason:
            audit.append({**row, "status": "rejected", "reason": reason, "residual_m": residual})
            continue

        viewer_global = compose_pose(align[row["viewer"]], vp)
        target_global = compose_pose(align[row["target"]], tp)
        target_yaw = target_global[2]
        tag_global_obs = transform_point(align[row["viewer"]], transform_point(vp, row["z"][:2]))
        target_xy_from_tag = tag_global_obs - rot2(target_yaw).dot(offsets[row["tag"]])
        target_global_from_tag = np.asarray([target_xy_from_tag[0], target_xy_from_tag[1], target_yaw], dtype=float)
        meas = relative_pose(viewer_global, target_global_from_tag)
        accepted.append(
            {
                "owner_robot": ROBOT_SYMBOLS[row["viewer"]],
                "from_robot": ROBOT_SYMBOLS[row["viewer"]],
                "from_g2o_key": int(v_node[1]),
                "to_robot": ROBOT_SYMBOLS[row["target"]],
                "to_g2o_key": int(t_node[1]),
                "x": meas[0],
                "y": meas[1],
                "theta": meas[2],
                "sigma_x": args.constraint_sigma_xy,
                "sigma_y": args.constraint_sigma_xy,
                "sigma_theta": args.constraint_sigma_theta,
            }
        )
        pair_counts[broad_pair] += 1
        last_pair_tag[pair_tag] = row["stamp"]
        audit.append({**row, "status": "accepted", "reason": "", "residual_m": residual})
    return accepted, audit


def write_inter_csv(path, rows):
    fields = [
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
    with open(path, "w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            out = dict(row)
            for key in ("x", "y", "theta", "sigma_x", "sigma_y", "sigma_theta"):
                out[key] = "%.9f" % float(out[key])
            writer.writerow(out)


def write_audit_csv(path, rows):
    fields = ["stamp", "target_stamp", "viewer", "target", "tag", "shift", "distance", "status", "reason", "residual_m"]
    with open(path, "w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", required=True)
    parser.add_argument("--detections-csv", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--robots", nargs="+", required=True)
    parser.add_argument("--tag-rule", action="append", required=True)
    parser.add_argument("--graph-dir", default=None)
    parser.add_argument("--rtabmap-dir", default=None)
    parser.add_argument("--min-distance", type=float, default=0.15)
    parser.add_argument("--max-distance", type=float, default=5.0)
    parser.add_argument("--max-odom-dt", type=float, default=0.20)
    parser.add_argument("--max-node-dt", type=float, default=0.75)
    parser.add_argument("--fit-sigma-xy", type=float, default=0.35)
    parser.add_argument("--max-tag-offset", type=float, default=1.50)
    parser.add_argument("--offset-excess-sigma", type=float, default=0.50)
    parser.add_argument("--max-nfev", type=int, default=500)
    parser.add_argument("--fit-min-separation", type=float, default=0.0)
    parser.add_argument("--fit-max-per-viewer-tag", type=int, default=0)
    parser.add_argument("--max-residual", type=float, default=0.65)
    parser.add_argument("--min-separation", type=float, default=2.0)
    parser.add_argument("--max-per-pair", type=int, default=80)
    parser.add_argument("--constraint-sigma-xy", type=float, default=0.75)
    parser.add_argument("--constraint-sigma-theta", type=float, default=3.141592653589793)
    return parser.parse_args()


def main():
    args = parse_args()
    os.makedirs(args.output_dir, exist_ok=True)
    graph_dir = args.graph_dir or os.path.join(args.scenario, "processed", "graphs")
    rtabmap_dir = args.rtabmap_dir or os.path.join(args.scenario, "processed", "rtabmap")
    rules = parse_tag_rules(args.tag_rule)

    odom = {}
    graph_nodes = {}
    for robot in args.robots:
        odom[robot] = read_tum(os.path.join(args.scenario, "eval", "tum", robot + "_odom.tum"))
        db_path = os.path.join(rtabmap_dir, robot, "rtabmap.db")
        g2o_path = os.path.join(graph_dir, robot + "_graph_poses.g2o")
        graph_nodes[robot] = read_graph_nodes(db_path, g2o_path)

    rows = read_detections(args.detections_csv, rules, args.robots, args.min_distance, args.max_distance)
    fit_rows = decimate_rows(rows, args.fit_min_separation, args.fit_max_per_viewer_tag)
    align, offsets, residual_rows, summary = fit_calibration(fit_rows, odom, args.robots, args)
    inter_rows, audit_rows = generate_constraints(residual_rows, align, offsets, graph_nodes, args)

    inter_csv = os.path.join(args.output_dir, "orkar_apriltag_mesa_inter_robot_constraints.csv")
    audit_csv = os.path.join(args.output_dir, "orkar_apriltag_mesa_constraint_audit.csv")
    write_inter_csv(inter_csv, inter_rows)
    write_audit_csv(audit_csv, audit_rows)
    summary.update(
        {
            "robots": args.robots,
            "tag_rules": rules,
            "detections_after_rule_filter": len(rows),
            "detections_used_for_fit": len(fit_rows),
            "accepted_constraints": len(inter_rows),
            "accepted_by_pair": {
                "%s->%s" % (key[0], key[1]): value
                for key, value in sorted(Counter((r["from_robot"], r["to_robot"]) for r in inter_rows).items())
            },
            "rejection_reasons": dict(Counter(r["reason"] for r in audit_rows if r["status"] != "accepted")),
            "inter_csv": inter_csv,
            "audit_csv": audit_csv,
            "parameters": vars(args),
        }
    )
    summary_json = os.path.join(args.output_dir, "orkar_apriltag_mesa_constraint_summary.json")
    with open(summary_json, "w") as handle:
        json.dump(summary, handle, indent=2)
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
