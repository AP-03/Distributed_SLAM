#!/usr/bin/env python
from __future__ import print_function

import math

import numpy as np
import rospy
import tf.transformations as tft
import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path
from phasespace_msgs.msg import Rigids
from std_msgs.msg import String

try:
    basestring
except NameError:
    basestring = (str,)


def _as_bool(value):
    if isinstance(value, bool):
        return value
    if isinstance(value, basestring):
        return value.strip().lower() in ["1", "true", "yes", "on"]
    return bool(value)


def _as_float_list(value, expected_len, name):
    if len(value) != expected_len:
        raise ValueError("%s must have %d values" % (name, expected_len))
    return [float(v) for v in value]


def _stamp_or_now(stamp):
    if stamp is None or stamp.to_sec() == 0.0:
        return rospy.Time.now()
    return stamp


def _normalise_quaternion(q):
    norm = math.sqrt(sum(v * v for v in q))
    if norm < 1e-9:
        return [0.0, 0.0, 0.0, 1.0]
    return [v / norm for v in q]


def _matrix_from_xyz_quat(xyz, quat_xyzw):
    quat_xyzw = _normalise_quaternion(quat_xyzw)
    transform = tft.quaternion_matrix(quat_xyzw)
    transform[0, 3] = xyz[0]
    transform[1, 3] = xyz[1]
    transform[2, 3] = xyz[2]
    return transform


def _matrix_from_xyz_rpy(xyz, rpy):
    quat = tft.quaternion_from_euler(rpy[0], rpy[1], rpy[2])
    return _matrix_from_xyz_quat(xyz, quat)


def _pose_from_matrix(transform, stamp, frame_id):
    quat = _normalise_quaternion(tft.quaternion_from_matrix(transform))
    pose = PoseStamped()
    pose.header.stamp = stamp
    pose.header.frame_id = frame_id
    pose.pose.position.x = transform[0, 3]
    pose.pose.position.y = transform[1, 3]
    pose.pose.position.z = transform[2, 3]
    pose.pose.orientation.x = quat[0]
    pose.pose.orientation.y = quat[1]
    pose.pose.orientation.z = quat[2]
    pose.pose.orientation.w = quat[3]
    return pose


def _param(defaults, robot_cfg, name, fallback):
    if name in robot_cfg:
        return robot_cfg[name]
    return defaults.get(name, fallback)


def _safe_publish(publisher, msg):
    try:
        publisher.publish(msg)
    except rospy.ROSException:
        if not rospy.is_shutdown():
            raise


class RigidRobotTracker(object):
    def __init__(self, robot_cfg, defaults, world_frame, tf_broadcaster):
        self.robot_id = str(robot_cfg["robot_id"])
        self.rigid_body_id = int(robot_cfg["rigid_body_id"])
        self.world_frame = str(robot_cfg.get("world_frame", world_frame))
        self.child_frame = str(robot_cfg.get("child_frame", "%s/gt_base_link" % self.robot_id))

        self.publish_tf = _as_bool(_param(defaults, robot_cfg, "publish_tf", True))
        self.publish_path = _as_bool(_param(defaults, robot_cfg, "publish_path", True))
        self.path_downsample = max(1, int(_param(defaults, robot_cfg, "path_downsample", 10)))
        self.max_path_length = int(_param(defaults, robot_cfg, "max_path_length", 20000))
        self.min_condition = float(_param(defaults, robot_cfg, "min_condition", 0.0))
        self.reject_zero_pose = _as_bool(_param(defaults, robot_cfg, "reject_zero_pose", True))
        self.position_covariance = float(_param(defaults, robot_cfg, "position_covariance", 0.0001))
        self.orientation_covariance = float(_param(defaults, robot_cfg, "orientation_covariance", 0.0001))

        rigid_to_base_xyz = _as_float_list(
            _param(defaults, robot_cfg, "rigid_to_base_xyz", [0.0, 0.0, 0.0]),
            3,
            "rigid_to_base_xyz",
        )
        if "rigid_to_base_quat_xyzw" in robot_cfg or "rigid_to_base_quat_xyzw" in defaults:
            rigid_to_base_quat = _as_float_list(
                _param(defaults, robot_cfg, "rigid_to_base_quat_xyzw", [0.0, 0.0, 0.0, 1.0]),
                4,
                "rigid_to_base_quat_xyzw",
            )
            self.t_rigid_base = _matrix_from_xyz_quat(rigid_to_base_xyz, rigid_to_base_quat)
        else:
            rigid_to_base_rpy = _as_float_list(
                _param(defaults, robot_cfg, "rigid_to_base_rpy", [0.0, 0.0, 0.0]),
                3,
                "rigid_to_base_rpy",
            )
            self.t_rigid_base = _matrix_from_xyz_rpy(rigid_to_base_xyz, rigid_to_base_rpy)

        self.pose_pub = rospy.Publisher("/gt/%s/pose" % self.robot_id, PoseStamped, queue_size=50)
        self.odom_pub = rospy.Publisher("/gt/%s/odom" % self.robot_id, Odometry, queue_size=50)
        self.path_pub = rospy.Publisher("/gt/%s/path" % self.robot_id, Path, queue_size=1, latch=True)
        self.status_pub = rospy.Publisher("/gt/%s/status" % self.robot_id, String, queue_size=5)
        self.tf_broadcaster = tf_broadcaster

        self.path = Path()
        self.path.header.frame_id = self.world_frame
        self.seen_count = 0
        self.tracked_count = 0
        self.rejected_count = 0
        self.last_pose_stamp = None
        self.last_status_time = rospy.Time(0)
        self.last_raw_time = None

        rospy.loginfo(
            "  %s rigid_body_id=%d child_frame=%s",
            self.robot_id,
            self.rigid_body_id,
            self.child_frame,
        )

    def process(self, rigid, stamp):
        self.seen_count += 1
        self.last_raw_time = rigid.time
        cond = float(rigid.cond)

        if cond < self.min_condition:
            self.rejected_count += 1
            self._publish_status(stamp, "low_condition", cond)
            return

        xyz = [float(rigid.x), float(rigid.y), float(rigid.z)]
        quat_xyzw = [float(rigid.qx), float(rigid.qy), float(rigid.qz), float(rigid.qw)]
        if not self._valid_pose(xyz, quat_xyzw):
            self.rejected_count += 1
            self._publish_status(stamp, "invalid_pose", cond)
            return

        stamp = self._monotonic_stamp(stamp)
        t_world_rigid = _matrix_from_xyz_quat(xyz, quat_xyzw)
        t_world_base = np.dot(t_world_rigid, self.t_rigid_base)
        pose = _pose_from_matrix(t_world_base, stamp, self.world_frame)

        self.tracked_count += 1
        _safe_publish(self.pose_pub, pose)
        _safe_publish(self.odom_pub, self._make_odom(pose))
        if self.publish_path and self.tracked_count % self.path_downsample == 0:
            self._publish_path(pose)
        if self.publish_tf:
            self._publish_tf(pose)
        self._publish_status(stamp, "tracking", cond)

    def mark_missing(self, stamp):
        self._publish_status(stamp, "missing", None)

    def _valid_pose(self, xyz, quat_xyzw):
        if not all(np.isfinite(value) for value in xyz + quat_xyzw):
            return False
        if self.reject_zero_pose and all(abs(value) < 1e-9 for value in xyz):
            return False
        return math.sqrt(sum(value * value for value in quat_xyzw)) > 1e-9

    def _monotonic_stamp(self, stamp):
        if self.last_pose_stamp is not None and stamp <= self.last_pose_stamp:
            stamp = self.last_pose_stamp + rospy.Duration.from_sec(0.000001)
        self.last_pose_stamp = stamp
        return stamp

    def _make_odom(self, pose):
        odom = Odometry()
        odom.header = pose.header
        odom.child_frame_id = self.child_frame
        odom.pose.pose = pose.pose
        odom.pose.covariance[0] = self.position_covariance
        odom.pose.covariance[7] = self.position_covariance
        odom.pose.covariance[14] = self.position_covariance
        odom.pose.covariance[21] = self.orientation_covariance
        odom.pose.covariance[28] = self.orientation_covariance
        odom.pose.covariance[35] = self.orientation_covariance
        return odom

    def _publish_path(self, pose):
        self.path.header.stamp = pose.header.stamp
        self.path.poses.append(pose)
        if self.max_path_length > 0 and len(self.path.poses) > self.max_path_length:
            self.path.poses = self.path.poses[-self.max_path_length:]
        _safe_publish(self.path_pub, self.path)

    def _publish_tf(self, pose):
        transform = TransformStamped()
        transform.header = pose.header
        transform.child_frame_id = self.child_frame
        transform.transform.translation.x = pose.pose.position.x
        transform.transform.translation.y = pose.pose.position.y
        transform.transform.translation.z = pose.pose.position.z
        transform.transform.rotation = pose.pose.orientation
        try:
            self.tf_broadcaster.sendTransform(transform)
        except rospy.ROSException:
            if not rospy.is_shutdown():
                raise

    def _publish_status(self, stamp, state, cond):
        if (stamp - self.last_status_time).to_sec() < 1.0:
            return
        self.last_status_time = stamp
        parts = [
            "robot_id=%s" % self.robot_id,
            "state=%s" % state,
            "rigid_body_id=%d" % self.rigid_body_id,
            "seen=%d" % self.seen_count,
            "tracked=%d" % self.tracked_count,
            "rejected=%d" % self.rejected_count,
        ]
        if cond is not None:
            parts.append("cond=%.4f" % cond)
        if self.last_raw_time is not None:
            parts.append("raw_time=%s" % self.last_raw_time)
        _safe_publish(self.status_pub, String(data=" ".join(parts)))


class MultiRigidGroundTruthNode(object):
    def __init__(self):
        rospy.init_node("multi_rigid_ground_truth")
        self.world_frame = rospy.get_param("~world_frame", rospy.get_param("world_frame", "mocap_world"))
        self.input_topic = rospy.get_param("~input_topic", rospy.get_param("input_topic", "/phasespace/stamped/rigids"))
        self.subscriber_queue_size = max(1, int(rospy.get_param("~subscriber_queue_size", rospy.get_param("subscriber_queue_size", 1))))

        defaults = rospy.get_param("~defaults", rospy.get_param("defaults", {}))
        robots = rospy.get_param("~robots", rospy.get_param("robots", []))
        if not robots:
            raise ValueError("multi_rigid_ground_truth requires a non-empty robots list")

        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.trackers = [RigidRobotTracker(robot_cfg, defaults, self.world_frame, self.tf_broadcaster) for robot_cfg in robots]
        self.trackers_by_rigid_id = {}
        for tracker in self.trackers:
            if tracker.rigid_body_id in self.trackers_by_rigid_id:
                raise ValueError("duplicate rigid_body_id %d" % tracker.rigid_body_id)
            self.trackers_by_rigid_id[tracker.rigid_body_id] = tracker

        self.sub = rospy.Subscriber(self.input_topic, Rigids, self._rigids_callback, queue_size=self.subscriber_queue_size)

        rospy.loginfo("Multi-robot PhaseSpace rigid ground truth started")
        rospy.loginfo("  robots=%s", ", ".join(tracker.robot_id for tracker in self.trackers))
        rospy.loginfo("  input=%s frame=%s", self.input_topic, self.world_frame)
        rospy.loginfo("  raw PhaseSpace markers/rigids should still be recorded for audit and offline recalibration")

    def _rigids_callback(self, msg):
        if rospy.is_shutdown():
            return
        stamp = _stamp_or_now(msg.header.stamp)
        matched_ids = set()
        for rigid in msg.rigids:
            tracker = self.trackers_by_rigid_id.get(int(rigid.id))
            if tracker is None:
                continue
            matched_ids.add(tracker.rigid_body_id)
            tracker.process(rigid, stamp)
        for tracker in self.trackers:
            if tracker.rigid_body_id not in matched_ids:
                tracker.mark_missing(stamp)

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        MultiRigidGroundTruthNode().run()
    except rospy.ROSInterruptException:
        pass
