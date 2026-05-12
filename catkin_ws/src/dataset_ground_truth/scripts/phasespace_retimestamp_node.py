#!/usr/bin/env python
from __future__ import print_function

import copy

import rospy
from phasespace_msgs.msg import Cameras, Markers, Rigids


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


class PhaseSpaceRestampNode(object):
    """Publish dataset-safe PhaseSpace topics with stamped headers.

    The upstream PhaseSpace bridge currently publishes container messages with a
    std_msgs/Header field but leaves the stamp/frame empty. Keeping this fix in
    our dataset package avoids losing it when the upstream submodule is updated.
    """

    def __init__(self):
        rospy.init_node("phasespace_retimestamp")

        self.frame_id = rospy.get_param("~frame_id", "mocap_world")
        self.preserve_nonzero_stamp = _as_bool(rospy.get_param("~preserve_nonzero_stamp", True))
        self.use_message_frame = _as_bool(rospy.get_param("~use_message_frame", False))
        self.queue_size = max(1, int(rospy.get_param("~queue_size", 100)))

        self.markers_in = rospy.get_param("~markers_in", "/phasespace/markers")
        self.rigids_in = rospy.get_param("~rigids_in", "/phasespace/rigids")
        self.cameras_in = rospy.get_param("~cameras_in", "/phasespace/cameras")

        self.markers_out = rospy.get_param("~markers_out", "/phasespace/stamped/markers")
        self.rigids_out = rospy.get_param("~rigids_out", "/phasespace/stamped/rigids")
        self.cameras_out = rospy.get_param("~cameras_out", "/phasespace/stamped/cameras")

        self.last_stamp = rospy.Time(0)

        self.markers_pub = rospy.Publisher(self.markers_out, Markers, queue_size=self.queue_size)
        self.rigids_pub = rospy.Publisher(self.rigids_out, Rigids, queue_size=self.queue_size)
        self.cameras_pub = rospy.Publisher(self.cameras_out, Cameras, queue_size=5, latch=True)

        self.markers_sub = rospy.Subscriber(self.markers_in, Markers, self._markers_callback, queue_size=self.queue_size)
        self.rigids_sub = rospy.Subscriber(self.rigids_in, Rigids, self._rigids_callback, queue_size=self.queue_size)
        self.cameras_sub = rospy.Subscriber(self.cameras_in, Cameras, self._cameras_callback, queue_size=5)

        rospy.loginfo("PhaseSpace restamp bridge started")
        rospy.loginfo("  frame_id=%s preserve_nonzero_stamp=%s", self.frame_id, self.preserve_nonzero_stamp)
        rospy.loginfo("  markers: %s -> %s", self.markers_in, self.markers_out)
        rospy.loginfo("  rigids:  %s -> %s", self.rigids_in, self.rigids_out)
        rospy.loginfo("  cameras: %s -> %s", self.cameras_in, self.cameras_out)

    def _stamp_for(self, msg):
        stamp = msg.header.stamp if self.preserve_nonzero_stamp else rospy.Time(0)
        if stamp is None or stamp.to_sec() == 0.0:
            stamp = rospy.Time.now()
        if stamp <= self.last_stamp:
            stamp = self.last_stamp + rospy.Duration.from_sec(0.000001)
        self.last_stamp = stamp
        return stamp

    def _frame_for(self, msg):
        if self.use_message_frame and msg.header.frame_id:
            return msg.header.frame_id
        return self.frame_id

    def _restamp(self, msg):
        out = copy.deepcopy(msg)
        out.header.stamp = self._stamp_for(msg)
        out.header.frame_id = self._frame_for(msg)
        return out

    def _markers_callback(self, msg):
        if not rospy.is_shutdown():
            self.markers_pub.publish(self._restamp(msg))

    def _rigids_callback(self, msg):
        if not rospy.is_shutdown():
            self.rigids_pub.publish(self._restamp(msg))

    def _cameras_callback(self, msg):
        if not rospy.is_shutdown():
            self.cameras_pub.publish(self._restamp(msg))

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        PhaseSpaceRestampNode().run()
    except rospy.ROSInterruptException:
        pass
