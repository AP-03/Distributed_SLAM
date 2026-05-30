#!/usr/bin/env python3
import argparse
import sqlite3


def read_stamps(db_path):
    con = sqlite3.connect(db_path)
    try:
        return {int(row[0]): float(row[1]) for row in con.execute("select id, stamp from Node")}
    finally:
        con.close()


def iter_g2o_vertices(g2o_path):
    with open(g2o_path) as f:
        for line in f:
            parts = line.strip().split()
            if not parts or parts[0] != "VERTEX_SE3:QUAT":
                continue
            if len(parts) < 9:
                continue
            yield (
                int(parts[1]),
                float(parts[2]),
                float(parts[3]),
                float(parts[4]),
                float(parts[5]),
                float(parts[6]),
                float(parts[7]),
                float(parts[8]),
            )


def main():
    parser = argparse.ArgumentParser(description="Export RTAB-Map g2o vertices to TUM using stamps from rtabmap.db.")
    parser.add_argument("--g2o", required=True, help="Input g2o file exported by rtabmap-export.")
    parser.add_argument("--db", required=True, help="RTAB-Map database containing Node id/stamp rows.")
    parser.add_argument("--output", required=True, help="Output TUM text file.")
    args = parser.parse_args()

    stamps = read_stamps(args.db)
    rows = []
    missing = 0
    for node_id, x, y, z, qx, qy, qz, qw in iter_g2o_vertices(args.g2o):
        stamp = stamps.get(node_id)
        if stamp is None:
            missing += 1
            continue
        rows.append((stamp, x, y, z, qx, qy, qz, qw))

    rows.sort(key=lambda r: r[0])
    with open(args.output, "w") as f:
        for row in rows:
            f.write("%.9f %.9f %.9f %.9f %.9f %.9f %.9f %.9f\n" % row)

    print("wrote %d poses to %s (%d vertices missing stamps)" % (len(rows), args.output, missing))


if __name__ == "__main__":
    main()
