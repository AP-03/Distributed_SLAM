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

try:
    basestring
except NameError:
    basestring = (str,)


def _as_float_list(value, expected_len, name):
    if len(value) != expected_len:
        raise ValueError("%s must have %d values" % (name, expected_len))
    return [float(v) for v in value]


def _as_bool(value):
    if isinstance(value, bool):
        return value
    if isinstance(value, basestring):
        return value.strip().lower() in ["1", "true", "yes", "on"]
    return bool(value)


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


class PhaseSpaceGroundTruthNode(object):
    def __init__(self):
        rospy.init_node("phasespace_ground_truth_node")

        self.robot_id = rospy.get_param("~robot_id", rospy.get_param("robot_id", "robot_01"))
        self.rigid_body_id = int(rospy.get_param("~rigid_body_id", rospy.get_param("rigid_body_id", 1)))
        self.world_frame = rospy.get_param("~world_frame", rospy.get_param("world_frame", "map"))
        default_child_frame = "%s/gt_base_link" % self.robot_id
        self.child_frame = rospy.get_param("~child_frame", rospy.get_param("child_frame", default_child_frame))
        self.input_topic = rospy.get_param("~input_topic", rospy.get_param("input_topic", "/phasespace/rigids"))

        self.pose_topic = rospy.get_param("~pose_topic", "/gt/%s/pose" % self.robot_id)
        self.odom_topic = rospy.get_param("~odom_topic", "/gt/%s/odom" % self.robot_id)
        self.path_topic = rospy.get_param("~path_topic", "/gt/%s/path" % self.robot_id)

        self.publish_tf = _as_bool(rospy.get_param("~publish_tf", rospy.get_param("publish_tf", True)))
        self.publish_path = _as_bool(rospy.get_param("~publish_path", rospy.get_param("publish_path", True)))
        self.path_downsample = max(1, int(rospy.get_param("~path_downsample", rospy.get_param("path_downsample", 1))))
        self.max_path_length = int(rospy.get_param("~max_path_length", rospy.get_param("max_path_length", 20000)))

        rigid_to_base_xyz = _as_float_list(
            rospy.get_param("~rigid_to_base_xyz", rospy.get_param("rigid_to_base_xyz", [0.0, 0.0, 0.0])),
            3,
            "rigid_to_base_xyz",
        )
        rigid_to_base_rpy = _as_float_list(
            rospy.get_param("~rigid_to_base_rpy", rospy.get_param("rigid_to_base_rpy", [0.0, 0.0, 0.0])),
            3,
            "rigid_to_base_rpy",
        )
        self.t_rigid_base = _matrix_from_xyz_rpy(rigid_to_base_xyz, rigid_to_base_rpy)

        self.position_covariance = float(rospy.get_param("~position_covariance", rospy.get_param("position_covariance", 0.0001)))
        self.orientation_covariance = float(rospy.get_param("~orientation_covariance", rospy.get_param("orientation_covariance", 0.0001)))

        self.pose_pub = rospy.Publisher(self.pose_topic, PoseStamped, queue_size=20)
        self.odom_pub = rospy.Publisher(self.odom_topic, Odometry, queue_size=20)
        self.path_pub = rospy.Publisher(self.path_topic, Path, queue_size=1, latch=True)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster() if self.publish_tf else None

        self.path = Path()
        self.path.header.frame_id = self.world_frame
        self.message_count = 0
        self.matched_count = 0
        self.last_raw_time = None

        self.sub = rospy.Subscriber(self.input_topic, Rigids, self._rigids_callback, queue_size=100)

        rospy.loginfo("PhaseSpace ground truth started")
        rospy.loginfo("  robot_id=%s rigid_body_id=%d", self.robot_id, self.rigid_body_id)
        rospy.loginfo("  input=%s pose=%s odom=%s path=%s", self.input_topic, self.pose_topic, self.odom_topic, self.path_topic)
        rospy.loginfo("  frame=%s child=%s publish_tf=%s", self.world_frame, self.child_frame, self.publish_tf)

    def _rigids_callback(self, msg):
        self.message_count += 1
        stamp = _stamp_or_now(msg.header.stamp)

        for rigid in msg.rigids:
            if rigid.id != self.rigid_body_id:
                continue

            self.matched_count += 1
            self.last_raw_time = rigid.time

            t_world_rigid = _matrix_from_xyz_quat(
                [rigid.x, rigid.y, rigid.z],
                [rigid.qx, rigid.qy, rigid.qz, rigid.qw],
            )
            t_world_base = np.dot(t_world_rigid, self.t_rigid_base)
            pose = _pose_from_matrix(t_world_base, stamp, self.world_frame)

            self.pose_pub.publish(pose)
            self.odom_pub.publish(self._make_odom(pose))

            if self.publish_path and self.matched_count % self.path_downsample == 0:
                self._publish_path(pose)

            if self.publish_tf:
                self._publish_tf(pose)
            return

        rospy.logwarn_throttle(5.0, "Rigid body id %d not present on %s", self.rigid_body_id, self.input_topic)

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
        self.path_pub.publish(self.path)

    def _publish_tf(self, pose):
        transform = TransformStamped()
        transform.header = pose.header
        transform.child_frame_id = self.child_frame
        transform.transform.translation.x = pose.pose.position.x
        transform.transform.translation.y = pose.pose.position.y
        transform.transform.translation.z = pose.pose.position.z
        transform.transform.rotation = pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        PhaseSpaceGroundTruthNode().run()
    except rospy.ROSInterruptException:
        pass
