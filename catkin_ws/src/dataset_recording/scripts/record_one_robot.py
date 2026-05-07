#!/usr/bin/env python
from __future__ import print_function

import os
import signal
import subprocess
import time

import rospy

try:
    import yaml
except ImportError:
    yaml = None

try:
    basestring
except NameError:
    basestring = (str,)


def _mkdir(path):
    if not os.path.isdir(path):
        os.makedirs(path)


def _safe_run(command):
    try:
        return subprocess.check_output(command, stderr=subprocess.STDOUT)
    except OSError as exc:
        return "Command not available: %s\n" % exc
    except subprocess.CalledProcessError as exc:
        return exc.output


def _param(name, default):
    return rospy.get_param("~" + name, rospy.get_param(name, default))


def _as_bool(value):
    if isinstance(value, bool):
        return value
    if isinstance(value, basestring):
        return value.strip().lower() in ["1", "true", "yes", "on"]
    return bool(value)


def _normalise_topics(topics):
    if isinstance(topics, basestring):
        topics = topics.replace(",", " ").split()
    return [str(topic) for topic in topics if str(topic).strip()]


def _write_metadata(path, metadata):
    if yaml is not None:
        with open(path, "w") as handle:
            yaml.safe_dump(metadata, handle, default_flow_style=False)
        return

    with open(path, "w") as handle:
        for key in sorted(metadata.keys()):
            handle.write("%s: %s\n" % (key, metadata[key]))


class OneRobotRecorder(object):
    def __init__(self):
        rospy.init_node("dataset_record_one_robot")

        self.robot_id = _param("robot_id", "robot_01")
        self.scenario = _param("scenario", "debug")
        default_run_id = "%s_%s_%s" % (time.strftime("%Y%m%d_%H%M%S"), self.scenario, self.robot_id)
        self.run_id = _param("run_id", default_run_id)
        self.output_root = os.path.expanduser(_param("output_root", "~/Distributed_SLAM/runs"))
        self.operator = _param("operator", os.environ.get("USER", "unknown"))
        self.notes = _param("notes", "")
        self.split = _as_bool(_param("split", False))
        self.split_size_mb = int(_param("split_size_mb", 4096))
        self.topics = _normalise_topics(_param("topics", []))

        if not self.topics:
            raise RuntimeError("No topics configured for recording")

        self.run_dir = os.path.join(self.output_root, self.run_id)
        self.raw_dir = os.path.join(self.run_dir, "raw")
        self.validation_dir = os.path.join(self.run_dir, "validation")
        self.calibration_dir = os.path.join(self.run_dir, "calibration")
        self.processed_dir = os.path.join(self.run_dir, "processed")
        self.bag_path = os.path.join(self.raw_dir, "%s.bag" % self.robot_id)
        self.process = None

    def prepare(self):
        for path in [self.raw_dir, self.validation_dir, self.calibration_dir, self.processed_dir]:
            _mkdir(path)

        metadata = {
            "run_id": self.run_id,
            "robot_id": self.robot_id,
            "scenario": self.scenario,
            "operator": self.operator,
            "notes": self.notes,
            "created_unix_time": time.time(),
            "created_local_time": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
            "ros_master_uri": os.environ.get("ROS_MASTER_URI", ""),
            "topics": self.topics,
            "raw_bag": os.path.relpath(self.bag_path, self.run_dir),
        }
        _write_metadata(os.path.join(self.run_dir, "metadata.yaml"), metadata)

        ntp_path = os.path.join(self.validation_dir, "ntp_status.txt")
        with open(ntp_path, "w") as handle:
            handle.write("$ chronyc tracking\n")
            handle.write(_safe_run(["chronyc", "tracking"]))
            handle.write("\n$ chronyc sources -v\n")
            handle.write(_safe_run(["chronyc", "sources", "-v"]))

    def start(self):
        command = ["rosbag", "record", "-O", self.bag_path]
        if self.split:
            command.extend(["--split", "--size=%d" % self.split_size_mb])
        command.extend(self.topics)

        rospy.loginfo("Recording run_id=%s robot_id=%s scenario=%s", self.run_id, self.robot_id, self.scenario)
        rospy.loginfo("Run directory: %s", self.run_dir)
        rospy.loginfo("Bag path: %s", self.bag_path)
        rospy.loginfo("Recording %d topics", len(self.topics))
        self.process = subprocess.Popen(command, preexec_fn=os.setsid)

    def stop(self):
        if self.process is None or self.process.poll() is not None:
            return
        rospy.loginfo("Stopping rosbag recorder")
        try:
            os.killpg(os.getpgid(self.process.pid), signal.SIGINT)
            self.process.wait()
        except OSError:
            pass

    def spin(self):
        self.prepare()
        self.start()
        rospy.on_shutdown(self.stop)
        while not rospy.is_shutdown():
            if self.process.poll() is not None:
                raise RuntimeError("rosbag record exited with status %s" % self.process.returncode)
            rospy.sleep(0.5)


if __name__ == "__main__":
    recorder = OneRobotRecorder()
    try:
        recorder.spin()
    finally:
        recorder.stop()
