#!/usr/bin/env python
from __future__ import print_function

import math

import numpy as np
import rospy
import tf.transformations as tft
import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path
from phasespace_msgs.msg import Markers
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


def _stamp_or_now(stamp):
    if stamp is None or stamp.to_sec() == 0.0:
        return rospy.Time.now()
    return stamp


def _normalise_quaternion(q):
    norm = math.sqrt(sum(v * v for v in q))
    if norm < 1e-9:
        return [0.0, 0.0, 0.0, 1.0]
    return [v / norm for v in q]


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


def _is_finite_vector(values):
    return all(np.isfinite(float(v)) for v in values)


def _parse_marker_positions(raw_positions):
    positions = {}
    if not raw_positions:
        return positions

    for marker_id, point in raw_positions.items():
        if len(point) != 3:
            raise ValueError("marker position for id %s must have 3 values" % marker_id)
        positions[int(marker_id)] = np.array([float(point[0]), float(point[1]), float(point[2])], dtype=float)
    return positions


def _estimate_transform(reference_points, observed_points, marker_ids):
    p = np.array([reference_points[marker_id] for marker_id in marker_ids], dtype=float)
    q = np.array([observed_points[marker_id] for marker_id in marker_ids], dtype=float)

    p_centroid = p.mean(axis=0)
    q_centroid = q.mean(axis=0)
    p_centered = p - p_centroid
    q_centered = q - q_centroid

    covariance = np.dot(p_centered.T, q_centered)
    u, singular_values, vt = np.linalg.svd(covariance)
    rotation = np.dot(vt.T, u.T)

    if np.linalg.det(rotation) < 0.0:
        vt[-1, :] *= -1.0
        rotation = np.dot(vt.T, u.T)

    translation = q_centroid - np.dot(rotation, p_centroid)
    transform = np.identity(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = translation

    predicted = np.dot(p, rotation.T) + translation
    residual_values = np.linalg.norm(predicted - q, axis=1)
    residuals = dict((marker_ids[index], float(residual_values[index])) for index in range(len(marker_ids)))
    rms = float(math.sqrt(np.mean(residual_values * residual_values)))

    return transform, rms, residuals, singular_values


class MarkerBootstrapGroundTruthNode(object):
    def __init__(self):
        rospy.init_node("marker_bootstrap_ground_truth_node")

        self.robot_id = rospy.get_param("~robot_id", rospy.get_param("robot_id", "robot_01"))
        self.world_frame = rospy.get_param("~world_frame", rospy.get_param("world_frame", "mocap_world"))
        default_child_frame = "%s/gt_base_link" % self.robot_id
        self.child_frame = rospy.get_param("~child_frame", rospy.get_param("child_frame", default_child_frame))
        self.input_topic = rospy.get_param("~input_topic", rospy.get_param("input_topic", "/phasespace/markers"))

        self.marker_ids = [int(marker_id) for marker_id in rospy.get_param("~marker_ids", rospy.get_param("marker_ids", [8, 9, 10, 11, 12, 13, 14, 15]))]
        self.marker_id_set = set(self.marker_ids)
        self.bootstrap_from_first_frame = _as_bool(rospy.get_param("~bootstrap_from_first_frame", rospy.get_param("bootstrap_from_first_frame", True)))
        self.bootstrap_min_markers = int(rospy.get_param("~bootstrap_min_markers", rospy.get_param("bootstrap_min_markers", len(self.marker_ids))))
        self.min_visible_markers = int(rospy.get_param("~min_visible_markers", rospy.get_param("min_visible_markers", 4)))
        self.min_condition = float(rospy.get_param("~min_condition", rospy.get_param("min_condition", 0.0)))
        self.reject_zero_positions = _as_bool(rospy.get_param("~reject_zero_positions", rospy.get_param("reject_zero_positions", True)))
        self.reject_outliers = _as_bool(rospy.get_param("~reject_outliers", rospy.get_param("reject_outliers", True)))
        self.max_marker_error = float(rospy.get_param("~max_marker_error", rospy.get_param("max_marker_error", 0.15)))
        self.subscriber_queue_size = max(1, int(rospy.get_param("~subscriber_queue_size", rospy.get_param("subscriber_queue_size", 1))))
        self.publish_rate = float(rospy.get_param("~publish_rate", rospy.get_param("publish_rate", 120.0)))
        self.min_publish_period = 0.0 if self.publish_rate <= 0.0 else 1.0 / self.publish_rate

        self.pose_topic = rospy.get_param("~pose_topic", "/gt/%s/pose" % self.robot_id)
        self.odom_topic = rospy.get_param("~odom_topic", "/gt/%s/odom" % self.robot_id)
        self.path_topic = rospy.get_param("~path_topic", "/gt/%s/path" % self.robot_id)
        self.status_topic = rospy.get_param("~status_topic", "/gt/%s/status" % self.robot_id)

        self.publish_tf = _as_bool(rospy.get_param("~publish_tf", rospy.get_param("publish_tf", True)))
        self.publish_path = _as_bool(rospy.get_param("~publish_path", rospy.get_param("publish_path", True)))
        self.path_downsample = max(1, int(rospy.get_param("~path_downsample", rospy.get_param("path_downsample", 10))))
        self.max_path_length = int(rospy.get_param("~max_path_length", rospy.get_param("max_path_length", 20000)))

        self.position_covariance = float(rospy.get_param("~position_covariance", rospy.get_param("position_covariance", 0.01)))
        self.orientation_covariance = float(rospy.get_param("~orientation_covariance", rospy.get_param("orientation_covariance", 0.05)))

        raw_marker_positions = rospy.get_param("~marker_positions", rospy.get_param("marker_positions", {}))
        self.reference_points = _parse_marker_positions(raw_marker_positions)
        if self.reference_points:
            self.bootstrap_from_first_frame = False

        self.pose_pub = rospy.Publisher(self.pose_topic, PoseStamped, queue_size=50)
        self.odom_pub = rospy.Publisher(self.odom_topic, Odometry, queue_size=50)
        self.path_pub = rospy.Publisher(self.path_topic, Path, queue_size=1, latch=True)
        self.status_pub = rospy.Publisher(self.status_topic, String, queue_size=5)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster() if self.publish_tf else None

        self.path = Path()
        self.path.header.frame_id = self.world_frame
        self.message_count = 0
        self.tracked_count = 0
        self.rejected_count = 0
        self.last_status_time = rospy.Time(0)
        self.last_visible_ids = []
        self.last_used_ids = []
        self.last_rms = None
        self.last_publish_stamp = None

        if not self.reference_points and not self.bootstrap_from_first_frame:
            raise ValueError("marker_positions are required when bootstrap_from_first_frame is false")

        self.sub = rospy.Subscriber(self.input_topic, Markers, self._markers_callback, queue_size=self.subscriber_queue_size)

        rospy.loginfo("Marker bootstrap ground truth started")
        rospy.loginfo("  robot_id=%s marker_ids=%s", self.robot_id, self.marker_ids)
        rospy.loginfo("  input=%s pose=%s odom=%s path=%s", self.input_topic, self.pose_topic, self.odom_topic, self.path_topic)
        rospy.loginfo("  frame=%s child=%s publish_tf=%s", self.world_frame, self.child_frame, self.publish_tf)
        rospy.loginfo("  publish_rate=%.3fHz subscriber_queue_size=%d", self.publish_rate, self.subscriber_queue_size)
        if self.reference_points:
            rospy.loginfo("  using configured marker geometry")
        else:
            rospy.logwarn("  using temporary bootstrapped LED-cluster geometry; do not use for final dataset metrics")

    def _markers_callback(self, msg):
        if rospy.is_shutdown():
            return

        self.message_count += 1
        stamp = _stamp_or_now(msg.header.stamp)
        observed = self._extract_observed_markers(msg)
        self.last_visible_ids = sorted(observed.keys())

        if not self.reference_points:
            self._try_bootstrap(observed)
            if not self.reference_points:
                rospy.logwarn_throttle(
                    2.0,
                    "Waiting to bootstrap marker rigid body: %d/%d configured LEDs visible",
                    len(observed),
                    self.bootstrap_min_markers,
                )
                self._publish_status(stamp, "waiting_for_bootstrap", observed.keys(), [], None)
                return

        available_ids = [marker_id for marker_id in self.marker_ids if marker_id in self.reference_points and marker_id in observed]
        if len(available_ids) < self.min_visible_markers:
            rospy.logwarn_throttle(
                2.0,
                "Only %d marker(s) visible for %s; need at least %d",
                len(available_ids),
                self.robot_id,
                self.min_visible_markers,
            )
            self._publish_status(stamp, "too_few_markers", observed.keys(), available_ids, None)
            return

        transform, rms, residuals, singular_values, used_ids = self._fit_with_optional_outlier_rejection(observed, available_ids)
        if len(used_ids) < self.min_visible_markers:
            rospy.logwarn_throttle(2.0, "Outlier rejection left too few markers for %s", self.robot_id)
            self._publish_status(stamp, "too_few_after_rejection", observed.keys(), used_ids, rms)
            return

        if not self._should_publish(stamp):
            return

        stamp = self._monotonic_output_stamp(stamp)
        self.tracked_count += 1
        self.last_used_ids = list(used_ids)
        self.last_rms = rms

        pose = _pose_from_matrix(transform, stamp, self.world_frame)
        self._safe_publish(self.pose_pub, pose)
        self._safe_publish(self.odom_pub, self._make_odom(pose))

        if self.publish_path and self.tracked_count % self.path_downsample == 0:
            self._publish_path(pose)

        if self.publish_tf:
            self._publish_tf(pose)

        self._publish_status(stamp, "tracking", observed.keys(), used_ids, rms, residuals, singular_values)

    def _extract_observed_markers(self, msg):
        observed = {}
        for marker in msg.markers:
            marker_id = int(marker.id)
            if marker_id not in self.marker_id_set:
                continue
            if marker.cond <= self.min_condition:
                continue
            point = [float(marker.x), float(marker.y), float(marker.z)]
            if not _is_finite_vector(point):
                continue
            if self.reject_zero_positions and abs(point[0]) < 1e-9 and abs(point[1]) < 1e-9 and abs(point[2]) < 1e-9:
                continue
            observed[marker_id] = np.array(point, dtype=float)
        return observed

    def _try_bootstrap(self, observed):
        if len(observed) < self.bootstrap_min_markers:
            return

        used_ids = [marker_id for marker_id in self.marker_ids if marker_id in observed]
        if len(used_ids) > self.bootstrap_min_markers:
            used_ids = used_ids[:self.bootstrap_min_markers]

        points = np.array([observed[marker_id] for marker_id in used_ids], dtype=float)
        centroid = points.mean(axis=0)
        self.reference_points = dict((marker_id, observed[marker_id] - centroid) for marker_id in used_ids)
        rospy.logwarn("Bootstrapped temporary marker rigid body from IDs %s", used_ids)
        rospy.logwarn("Temporary GT origin is the LED centroid, not robot base_link")

    def _fit_with_optional_outlier_rejection(self, observed, available_ids):
        used_ids = list(available_ids)
        transform, rms, residuals, singular_values = _estimate_transform(self.reference_points, observed, used_ids)

        if not self.reject_outliers or self.max_marker_error <= 0.0:
            return transform, rms, residuals, singular_values, used_ids

        while len(used_ids) > self.min_visible_markers:
            worst_id = max(residuals.keys(), key=lambda marker_id: residuals[marker_id])
            if residuals[worst_id] <= self.max_marker_error:
                break
            used_ids.remove(worst_id)
            self.rejected_count += 1
            transform, rms, residuals, singular_values = _estimate_transform(self.reference_points, observed, used_ids)

        return transform, rms, residuals, singular_values, used_ids

    def _should_publish(self, stamp):
        if self.last_publish_stamp is None or self.min_publish_period <= 0.0:
            return True
        return (stamp - self.last_publish_stamp).to_sec() >= self.min_publish_period

    def _monotonic_output_stamp(self, stamp):
        if self.last_publish_stamp is not None and stamp <= self.last_publish_stamp:
            stamp = self.last_publish_stamp + rospy.Duration.from_sec(0.000001)
        self.last_publish_stamp = stamp
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
        self._safe_publish(self.path_pub, self.path)

    def _publish_tf(self, pose):
        if rospy.is_shutdown():
            return
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

    def _publish_status(self, stamp, state, visible_ids, used_ids, rms, residuals=None, singular_values=None):
        if rospy.is_shutdown():
            return
        if (stamp - self.last_status_time).to_sec() < 1.0:
            return
        self.last_status_time = stamp

        parts = [
            "state=%s" % state,
            "visible=%s" % ",".join(str(marker_id) for marker_id in sorted(visible_ids)),
            "used=%s" % ",".join(str(marker_id) for marker_id in sorted(used_ids)),
            "tracked=%d" % self.tracked_count,
            "rejected=%d" % self.rejected_count,
        ]
        if rms is not None:
            parts.append("rms=%.4f" % rms)
        if residuals:
            worst_id = max(residuals.keys(), key=lambda marker_id: residuals[marker_id])
            parts.append("worst=%d:%.4f" % (worst_id, residuals[worst_id]))
        if singular_values is not None and len(singular_values) >= 3:
            parts.append("sv=%.4g,%.4g,%.4g" % (singular_values[0], singular_values[1], singular_values[2]))

        self._safe_publish(self.status_pub, String(data=" ".join(parts)))

    def _safe_publish(self, publisher, msg):
        if rospy.is_shutdown():
            return
        try:
            publisher.publish(msg)
        except rospy.ROSException:
            if not rospy.is_shutdown():
                raise

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        MarkerBootstrapGroundTruthNode().run()
    except rospy.ROSInterruptException:
        pass
