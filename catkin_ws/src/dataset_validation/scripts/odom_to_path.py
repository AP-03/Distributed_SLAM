#!/usr/bin/env python
from __future__ import print_function

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path


class OdomToPath(object):
    def __init__(self):
        self.odom_topic = rospy.get_param("~odom_topic", "/odom")
        self.path_topic = rospy.get_param("~path_topic", "/odom/path")
        self.max_length = int(rospy.get_param("~max_length", 10000))
        self.downsample = max(1, int(rospy.get_param("~downsample", 1)))

        self.path = Path()
        self.count = 0
        self.pub = rospy.Publisher(self.path_topic, Path, queue_size=1, latch=True)
        self.sub = rospy.Subscriber(self.odom_topic, Odometry, self._callback, queue_size=100)

        rospy.loginfo("Odom path visualizer: %s -> %s", self.odom_topic, self.path_topic)

    def _callback(self, msg):
        self.count += 1
        if self.count % self.downsample != 0:
            return

        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose

        self.path.header = msg.header
        self.path.poses.append(pose)
        if self.max_length > 0 and len(self.path.poses) > self.max_length:
            self.path.poses = self.path.poses[-self.max_length:]

        self.pub.publish(self.path)


def main():
    rospy.init_node("odom_to_path")
    OdomToPath()
    rospy.spin()


if __name__ == "__main__":
    main()
