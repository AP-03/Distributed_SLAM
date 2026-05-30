#!/usr/bin/env python3
import argparse
import csv
import json
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description="Write robot/GT/offset CSV from an ORKAR-style time-shift summary.")
    parser.add_argument("--summary", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()

    summary = json.loads(Path(args.summary).read_text())
    rows = summary["best_assignment"]["rows"]
    out = Path(args.output)
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("w", newline="") as handle:
        writer = csv.DictWriter(
            handle,
            fieldnames=["robot", "gt", "offset_s", "se2_rmse_m", "se2_median_m", "matched_poses"],
        )
        writer.writeheader()
        for row in rows:
            writer.writerow(
                {
                    "robot": row["robot"],
                    "gt": row["gt"],
                    "offset_s": row["gt_time_shift_s"],
                    "se2_rmse_m": row["se2_rmse_m"],
                    "se2_median_m": row["se2_median_m"],
                    "matched_poses": row["matched_poses"],
                }
            )
    print("wrote %s" % out)


if __name__ == "__main__":
    main()
