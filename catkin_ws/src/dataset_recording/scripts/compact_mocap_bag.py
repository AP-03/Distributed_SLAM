#!/usr/bin/env python
from __future__ import print_function

import argparse
import os
import re
import sys
import time

import rosbag


DEFAULT_DROP_PATTERNS = [
    r"^/phasespace/markers$",
    r"^/phasespace/stamped/markers$",
    r"^/mocap_debug/(led_markers|led_paths|rigid_markers|rigid_paths)$",
]


def _compile_patterns(patterns):
    return [re.compile(pattern) for pattern in patterns]


def _drop_topic(topic, patterns):
    return any(pattern.match(topic) for pattern in patterns)


def _format_size(num_bytes):
    value = float(num_bytes)
    for unit in ["B", "KB", "MB", "GB", "TB"]:
        if value < 1024.0 or unit == "TB":
            return "%.2f %s" % (value, unit)
        value /= 1024.0


def _output_path(input_path, output_path):
    if output_path:
        return output_path
    root, ext = os.path.splitext(input_path)
    if ext != ".bag":
        ext = ".bag"
    return root + "_compact" + ext


def compact_bag(input_path, output_path, drop_patterns, force=False):
    output_path = _output_path(input_path, output_path)
    if os.path.abspath(input_path) == os.path.abspath(output_path):
        raise RuntimeError("output bag must be different from input bag")
    if os.path.exists(output_path) and not force:
        raise RuntimeError("output exists; pass --force to overwrite: %s" % output_path)

    output_dir = os.path.dirname(os.path.abspath(output_path))
    if output_dir and not os.path.isdir(output_dir):
        os.makedirs(output_dir)

    patterns = _compile_patterns(drop_patterns)
    written = 0
    dropped = 0
    written_by_topic = {}
    dropped_by_topic = {}
    start = time.time()

    with rosbag.Bag(input_path, "r") as source:
        with rosbag.Bag(output_path, "w") as target:
            for topic, msg, stamp in source.read_messages(raw=False):
                if _drop_topic(topic, patterns):
                    dropped += 1
                    dropped_by_topic[topic] = dropped_by_topic.get(topic, 0) + 1
                    continue
                target.write(topic, msg, stamp)
                written += 1
                written_by_topic[topic] = written_by_topic.get(topic, 0) + 1

    elapsed = time.time() - start
    input_size = os.path.getsize(input_path)
    output_size = os.path.getsize(output_path)

    print("input:  %s (%s)" % (input_path, _format_size(input_size)))
    print("output: %s (%s)" % (output_path, _format_size(output_size)))
    print("kept_messages: %d" % written)
    print("dropped_messages: %d" % dropped)
    print("elapsed_sec: %.1f" % elapsed)
    print("saved: %s" % _format_size(max(0, input_size - output_size)))
    if dropped_by_topic:
        print("dropped_topics:")
        for topic in sorted(dropped_by_topic):
            print("  %s %d" % (topic, dropped_by_topic[topic]))
    return output_path


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Write a smaller mocap bag by dropping raw marker-heavy topics."
    )
    parser.add_argument("input_bag", help="input .bag")
    parser.add_argument(
        "output_bag",
        nargs="?",
        help="output .bag; default is INPUT_compact.bag",
    )
    parser.add_argument(
        "--drop",
        action="append",
        default=[],
        help="extra regex topic pattern to drop; may be repeated",
    )
    parser.add_argument("--force", action="store_true", help="overwrite output bag")
    return parser.parse_args(argv)


def main(argv):
    args = parse_args(argv)
    drop_patterns = list(DEFAULT_DROP_PATTERNS) + list(args.drop)
    compact_bag(args.input_bag, args.output_bag, drop_patterns, force=args.force)


if __name__ == "__main__":
    main(sys.argv[1:])
