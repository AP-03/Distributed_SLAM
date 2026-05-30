#!/usr/bin/env python3
import argparse
import json
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description="Write --tag-rule args from an ORKAR AprilTag diagnostic summary.")
    parser.add_argument("--summary", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--source", choices=["best_unique_by_tag", "best_by_tag"], default="best_unique_by_tag")
    parser.add_argument("--shift", default="0", help="Target clock shift to use in every tag rule.")
    args = parser.parse_args()

    summary = json.loads(Path(args.summary).read_text())
    rules = []
    for tag_text, row in sorted(summary[args.source].items(), key=lambda item: int(item[0])):
        target = row.get("target_robot")
        if not target:
            continue
        rules.append("--tag-rule %s:%s:%s" % (tag_text, target, args.shift))

    out = Path(args.output)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(" ".join(rules) + "\n")
    print("wrote %d tag rules to %s" % (len(rules), out))


if __name__ == "__main__":
    main()
