#!/usr/bin/env python
from __future__ import print_function

import math

import rospy
from geometry_msgs.msg import Point
from sensor_msgs.msg import Imu
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


def _color(r, g, b, a):
    c = ColorRGBA()
    c.r = r
    c.g = g
    c.b = b
    c.a = a
    return c


def _point(x, y, z):
    p = Point()
    p.x = x
    p.y = y
    p.z = z
    return p


class ImuMarkerVisualizer(object):
    def __init__(self):
        self.imu_topic = rospy.get_param("~imu_topic", "/imu")
        self.marker_topic = rospy.get_param("~marker_topic", "/imu/markers")
        self.fallback_frame = rospy.get_param("~fallback_frame", "imu_link")
        self.accel_scale = float(rospy.get_param("~accel_scale", 0.08))
        self.gyro_scale = float(rospy.get_param("~gyro_scale", 0.5))

        self.pub = rospy.Publisher(self.marker_topic, MarkerArray, queue_size=1)
        self.sub = rospy.Subscriber(self.imu_topic, Imu, self._callback, queue_size=50)
        rospy.loginfo("IMU marker visualizer: %s -> %s", self.imu_topic, self.marker_topic)

    def _arrow(self, msg, marker_id, name, start, vector, scale, color):
        marker = Marker()
        marker.header.stamp = msg.header.stamp
        marker.header.frame_id = msg.header.frame_id or self.fallback_frame
        marker.ns = name
        marker.id = marker_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.lifetime = rospy.Duration(0.25)
        marker.scale.x = 0.025
        marker.scale.y = 0.06
        marker.scale.z = 0.08
        marker.color = color

        end = _point(
            start.x + vector[0] * scale,
            start.y + vector[1] * scale,
            start.z + vector[2] * scale,
        )
        marker.points = [start, end]
        return marker

    def _text(self, msg, accel_norm, gyro_norm):
        marker = Marker()
        marker.header.stamp = msg.header.stamp
        marker.header.frame_id = msg.header.frame_id or self.fallback_frame
        marker.ns = "imu_text"
        marker.id = 10
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.lifetime = rospy.Duration(0.25)
        marker.pose.position.z = 0.35
        marker.scale.z = 0.12
        marker.color = _color(1.0, 1.0, 1.0, 0.9)
        marker.text = "IMU  accel %.2f m/s^2  gyro %.2f rad/s" % (accel_norm, gyro_norm)
        return marker

    def _callback(self, msg):
        accel = (
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
        )
        gyro = (
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z,
        )
        accel_norm = math.sqrt(accel[0] ** 2 + accel[1] ** 2 + accel[2] ** 2)
        gyro_norm = math.sqrt(gyro[0] ** 2 + gyro[1] ** 2 + gyro[2] ** 2)

        markers = MarkerArray()
        markers.markers.append(
            self._arrow(msg, 0, "imu_accel", _point(0.0, 0.0, 0.0), accel, self.accel_scale, _color(1.0, 0.45, 0.0, 0.9))
        )
        markers.markers.append(
            self._arrow(msg, 1, "imu_gyro", _point(0.0, 0.0, 0.15), gyro, self.gyro_scale, _color(0.0, 0.8, 1.0, 0.9))
        )
        markers.markers.append(self._text(msg, accel_norm, gyro_norm))
        self.pub.publish(markers)


def main():
    rospy.init_node("imu_marker_visualizer")
    ImuMarkerVisualizer()
    rospy.spin()


if __name__ == "__main__":
    main()
