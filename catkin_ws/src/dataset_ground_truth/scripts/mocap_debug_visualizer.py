#!/usr/bin/env python
from __future__ import print_function

import colorsys
import math
from collections import defaultdict

import rospy
from geometry_msgs.msg import Point
from phasespace_msgs.msg import Markers, Rigids
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


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


def _frame_from_header(header, fallback, use_message_frame):
    if use_message_frame and header.frame_id:
        return header.frame_id
    return fallback


def _point(x, y, z):
    p = Point()
    p.x = x
    p.y = y
    p.z = z
    return p


def _color_for_id(identifier, alpha=1.0):
    hue = ((int(identifier) * 0.61803398875) % 1.0)
    r, g, b = colorsys.hsv_to_rgb(hue, 0.78, 1.0)
    return r, g, b, alpha


def _distance(a, b):
    dx = a.x - b.x
    dy = a.y - b.y
    dz = a.z - b.z
    return math.sqrt(dx * dx + dy * dy + dz * dz)


class MocapDebugVisualizer(object):
    def __init__(self):
        rospy.init_node("mocap_debug_visualizer")

        self.markers_topic = rospy.get_param("~markers_topic", "/phasespace/markers")
        self.rigids_topic = rospy.get_param("~rigids_topic", "/phasespace/rigids")
        self.fixed_frame = rospy.get_param("~fixed_frame", "mocap_world")
        self.use_message_frame = _as_bool(rospy.get_param("~use_message_frame", True))
        self.max_path_points = int(rospy.get_param("~max_path_points", 2000))
        self.path_min_distance = float(rospy.get_param("~path_min_distance", 0.005))
        self.min_condition = float(rospy.get_param("~min_condition", 0.0))
        self.include_invalid = _as_bool(rospy.get_param("~include_invalid", False))
        self.sphere_scale = float(rospy.get_param("~sphere_scale", 0.035))
        self.rigid_scale = float(rospy.get_param("~rigid_scale", 0.25))
        self.path_width = float(rospy.get_param("~path_width", 0.01))
        self.label_scale = float(rospy.get_param("~label_scale", 0.06))
        self.marker_lifetime = float(rospy.get_param("~marker_lifetime", 0.5))
        self.publish_labels = _as_bool(rospy.get_param("~publish_labels", True))

        self.led_paths = defaultdict(list)
        self.rigid_paths = defaultdict(list)
        self.last_marker_ids = []
        self.last_rigid_ids = []

        self.led_markers_pub = rospy.Publisher("/mocap_debug/led_markers", MarkerArray, queue_size=5)
        self.led_paths_pub = rospy.Publisher("/mocap_debug/led_paths", MarkerArray, queue_size=5, latch=True)
        self.rigid_markers_pub = rospy.Publisher("/mocap_debug/rigid_markers", MarkerArray, queue_size=5)
        self.rigid_paths_pub = rospy.Publisher("/mocap_debug/rigid_paths", MarkerArray, queue_size=5, latch=True)
        self.status_pub = rospy.Publisher("/mocap_debug/status", String, queue_size=1, latch=True)

        self.markers_sub = rospy.Subscriber(self.markers_topic, Markers, self._markers_callback, queue_size=50)
        self.rigids_sub = rospy.Subscriber(self.rigids_topic, Rigids, self._rigids_callback, queue_size=50)
        self.status_timer = rospy.Timer(rospy.Duration(1.0), self._publish_status)

        rospy.loginfo("Mocap debug visualizer started")
        rospy.loginfo("  markers=%s rigids=%s frame=%s", self.markers_topic, self.rigids_topic, self.fixed_frame)
        rospy.loginfo("  outputs=/mocap_debug/led_markers /mocap_debug/led_paths /mocap_debug/rigid_markers /mocap_debug/rigid_paths")

    def _append_path_point(self, path_store, identifier, point):
        path = path_store[int(identifier)]
        if path and _distance(path[-1], point) < self.path_min_distance:
            return
        path.append(point)
        if self.max_path_points > 0 and len(path) > self.max_path_points:
            del path[:-self.max_path_points]

    def _valid_sample(self, x, y, z, cond):
        if self.include_invalid:
            return True
        if cond <= self.min_condition:
            return False
        return abs(x) > 1e-9 or abs(y) > 1e-9 or abs(z) > 1e-9

    def _markers_callback(self, msg):
        if rospy.is_shutdown():
            return
        stamp = _stamp_or_now(msg.header.stamp)
        frame_id = _frame_from_header(msg.header, self.fixed_frame, self.use_message_frame)
        markers = MarkerArray()
        paths = MarkerArray()
        self.last_marker_ids = []

        for led in msg.markers:
            if not self._valid_sample(led.x, led.y, led.z, led.cond):
                continue
            identifier = int(led.id)
            self.last_marker_ids.append(identifier)
            point = _point(led.x, led.y, led.z)
            self._append_path_point(self.led_paths, identifier, point)
            r, g, b, alpha = _color_for_id(identifier, 1.0 if led.cond >= 0 else 0.35)

            sphere = self._base_marker(stamp, frame_id, "led", identifier, Marker.SPHERE)
            sphere.pose.position = point
            sphere.scale.x = self.sphere_scale
            sphere.scale.y = self.sphere_scale
            sphere.scale.z = self.sphere_scale
            sphere.color.r = r
            sphere.color.g = g
            sphere.color.b = b
            sphere.color.a = alpha
            markers.markers.append(sphere)

            if self.publish_labels:
                label = self._base_marker(stamp, frame_id, "led_label", identifier, Marker.TEXT_VIEW_FACING)
                label.pose.position = _point(led.x, led.y, led.z + self.sphere_scale * 1.8)
                label.scale.z = self.label_scale
                label.color.r = r
                label.color.g = g
                label.color.b = b
                label.color.a = 1.0
                label.text = "LED %d" % identifier
                markers.markers.append(label)

        for identifier in sorted(self.led_paths.keys()):
            paths.markers.append(self._path_marker(stamp, frame_id, "led_path", identifier, self.led_paths[identifier]))

        self._safe_publish(self.led_markers_pub, markers)
        self._safe_publish(self.led_paths_pub, paths)

    def _rigids_callback(self, msg):
        if rospy.is_shutdown():
            return
        stamp = _stamp_or_now(msg.header.stamp)
        frame_id = _frame_from_header(msg.header, self.fixed_frame, self.use_message_frame)
        markers = MarkerArray()
        paths = MarkerArray()
        self.last_rigid_ids = []

        for rigid in msg.rigids:
            if not self._valid_sample(rigid.x, rigid.y, rigid.z, rigid.cond):
                continue
            identifier = int(rigid.id)
            self.last_rigid_ids.append(identifier)
            point = _point(rigid.x, rigid.y, rigid.z)
            self._append_path_point(self.rigid_paths, identifier, point)
            r, g, b, alpha = _color_for_id(identifier + 1000, 1.0)

            arrow = self._base_marker(stamp, frame_id, "rigid", identifier, Marker.ARROW)
            arrow.pose.position = point
            arrow.pose.orientation.x = rigid.qx
            arrow.pose.orientation.y = rigid.qy
            arrow.pose.orientation.z = rigid.qz
            arrow.pose.orientation.w = rigid.qw
            arrow.scale.x = self.rigid_scale
            arrow.scale.y = self.rigid_scale * 0.25
            arrow.scale.z = self.rigid_scale * 0.25
            arrow.color.r = r
            arrow.color.g = g
            arrow.color.b = b
            arrow.color.a = alpha
            markers.markers.append(arrow)

            if self.publish_labels:
                label = self._base_marker(stamp, frame_id, "rigid_label", identifier, Marker.TEXT_VIEW_FACING)
                label.pose.position = _point(rigid.x, rigid.y, rigid.z + self.rigid_scale * 0.6)
                label.scale.z = self.label_scale * 1.2
                label.color.r = r
                label.color.g = g
                label.color.b = b
                label.color.a = 1.0
                label.text = "Rigid %d" % identifier
                markers.markers.append(label)

        for identifier in sorted(self.rigid_paths.keys()):
            paths.markers.append(self._path_marker(stamp, frame_id, "rigid_path", identifier, self.rigid_paths[identifier]))

        self._safe_publish(self.rigid_markers_pub, markers)
        self._safe_publish(self.rigid_paths_pub, paths)

    def _base_marker(self, stamp, frame_id, namespace, identifier, marker_type):
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = frame_id
        marker.ns = namespace
        marker.id = int(identifier)
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.lifetime = rospy.Duration(self.marker_lifetime)
        marker.pose.orientation.w = 1.0
        return marker

    def _path_marker(self, stamp, frame_id, namespace, identifier, points):
        marker = self._base_marker(stamp, frame_id, namespace, identifier, Marker.LINE_STRIP)
        marker.lifetime = rospy.Duration(0.0)
        marker.scale.x = self.path_width
        r, g, b, alpha = _color_for_id(identifier, 0.9)
        if namespace.startswith("rigid"):
            r, g, b, alpha = _color_for_id(identifier + 1000, 0.9)
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = alpha
        marker.points = list(points)
        return marker

    def _publish_status(self, event):
        if rospy.is_shutdown():
            return
        text = "LED ids=%s | rigid ids=%s | led_paths=%d | rigid_paths=%d" % (
            ",".join(str(i) for i in sorted(self.last_marker_ids)) or "none",
            ",".join(str(i) for i in sorted(self.last_rigid_ids)) or "none",
            len(self.led_paths),
            len(self.rigid_paths),
        )
        self._safe_publish(self.status_pub, String(text))

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
        MocapDebugVisualizer().run()
    except rospy.ROSInterruptException:
        pass
