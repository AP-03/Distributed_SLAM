#!/usr/bin/env python
"""
gt_comparison_node.py
---------------------
ROS node that compares ground truth (PhaseSpace mocap) with robot odometry
in real-time during bag playback. Publishes:
  - /gt/path          : Green path trace of ground truth trajectory
  - /gt/pose          : Current GT pose (PoseStamped)
  - /gt/marker        : Green arrow marker at current GT pose
  - /odom/path        : Red path trace of odometry trajectory
  - /odom/marker      : Red arrow marker at current odom pose
  - /error/marker     : Yellow line connecting GT and odom showing error
  - /error/text       : Text overlay showing current positional error
  - TF: map -> odom   : Aligns both trajectories in the same frame

Works with ROS Melodic (Python 2.7 compatible).
"""

import rospy
import math
import numpy as np
from geometry_msgs.msg import PoseStamped, TransformStamped, Point
from nav_msgs.msg import Path, Odometry
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import tf2_ros
import tf.transformations as tft


class GTComparisonNode(object):

    def __init__(self):
        rospy.init_node('gt_comparison_node', anonymous=False)

        # --- Parameters ---
        self.rigid_id = rospy.get_param('~rigid_body_id', 1)
        self.gt_frame = rospy.get_param('~gt_frame', 'map')
        self.odom_frame = rospy.get_param('~odom_frame', 'odom')
        self.robot_frame = rospy.get_param('~robot_frame', 'base_link')
        self.auto_align = rospy.get_param('~auto_align', True)
        self.path_downsample = rospy.get_param('~path_downsample', 3)

        # Mocap axes might differ from ROS convention
        # PhaseSpace: +X right, +Y up, +Z toward camera (varies by setup)
        # ROS: +X forward, +Y left, +Z up
        # Set these params to remap if needed (defaults assume mocap already in ROS convention)
        self.swap_yz = rospy.get_param('~swap_mocap_yz', False)

        # --- State ---
        self.gt_path = Path()
        self.gt_path.header.frame_id = self.gt_frame
        self.odom_path = Path()
        self.odom_path.header.frame_id = self.odom_frame

        self.last_gt_pose = None       # (x, y, z, qx, qy, qz, qw)
        self.last_odom_pose = None     # (x, y, z, qx, qy, qz, qw)
        self.first_gt_pose = None
        self.first_odom_pose = None
        self.alignment_computed = False
        self.alignment_tf = np.eye(4)  # 4x4 homogeneous: map -> odom

        self.gt_msg_count = 0
        self.odom_msg_count = 0

        # --- Publishers ---
        self.gt_path_pub = rospy.Publisher('/gt/path', Path, queue_size=1, latch=True)
        self.gt_pose_pub = rospy.Publisher('/gt/pose', PoseStamped, queue_size=1)
        self.gt_marker_pub = rospy.Publisher('/gt/marker', Marker, queue_size=1)
        self.odom_path_pub = rospy.Publisher('/odom/path', Path, queue_size=1, latch=True)
        self.odom_marker_pub = rospy.Publisher('/odom/marker', Marker, queue_size=1)
        self.error_marker_pub = rospy.Publisher('/error/marker', Marker, queue_size=1)
        self.error_text_pub = rospy.Publisher('/error/text', Marker, queue_size=1)

        # --- TF broadcaster for map->odom alignment ---
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster()

        # --- Subscribers ---
        # We import the message dynamically to avoid hard dependency on phasespace_msgs
        try:
            from phasespace_msgs.msg import Rigids
            self.gt_sub = rospy.Subscriber('/phasespace/rigids', Rigids,
                                           self.gt_callback, queue_size=50)
            rospy.loginfo("Subscribed to /phasespace/rigids (phasespace_msgs/Rigids)")
        except ImportError:
            rospy.logwarn("phasespace_msgs not found! Using generic subscriber approach.")
            from rospy.msg import AnyMsg
            self.gt_sub = rospy.Subscriber('/phasespace/rigids', AnyMsg,
                                           self.gt_callback_generic, queue_size=50)

        self.odom_sub = rospy.Subscriber('/odom', Odometry,
                                          self.odom_callback, queue_size=50)

        # Timer to publish alignment TF at steady rate
        self.tf_timer = rospy.Timer(rospy.Duration(0.05), self.publish_alignment_tf)

        rospy.loginfo("GT Comparison Node started.")
        rospy.loginfo("  Rigid body ID: %d", self.rigid_id)
        rospy.loginfo("  Auto-align: %s", self.auto_align)
        rospy.loginfo("  GT frame: %s | Odom frame: %s", self.gt_frame, self.odom_frame)

    # -------------------------------------------------------------------------
    # Ground Truth (Mocap) Callback
    # -------------------------------------------------------------------------
    def gt_callback(self, msg):
        """Process phasespace_msgs/Rigids message."""
        for rigid in msg.rigids:
            if rigid.id == self.rigid_id:
                x, y, z = rigid.x, rigid.y, rigid.z
                qw, qx, qy, qz = rigid.qw, rigid.qx, rigid.qy, rigid.qz

                # Optional axis swap (PhaseSpace Y-up -> ROS Z-up)
                if self.swap_yz:
                    x, y, z = x, -z, y
                    qx, qy, qz, qw = qx, -qz, qy, qw

                # Convert to meters if PhaseSpace outputs mm (common)
                # Uncomment next line if your mocap outputs in mm:
                # x, y, z = x / 1000.0, y / 1000.0, z / 1000.0

                stamp = msg.header.stamp
                if stamp.secs == 0:
                    stamp = rospy.Time.now()

                self.last_gt_pose = (x, y, z, qx, qy, qz, qw)

                # Store first GT pose for alignment
                if self.first_gt_pose is None:
                    self.first_gt_pose = self.last_gt_pose
                    rospy.loginfo("First GT pose captured: (%.3f, %.3f, %.3f)", x, y, z)
                    self._try_compute_alignment()

                # Downsample path points
                self.gt_msg_count += 1
                if self.gt_msg_count % self.path_downsample == 0:
                    ps = PoseStamped()
                    ps.header.stamp = stamp
                    ps.header.frame_id = self.gt_frame
                    ps.pose.position.x = x
                    ps.pose.position.y = y
                    ps.pose.position.z = z
                    ps.pose.orientation.x = qx
                    ps.pose.orientation.y = qy
                    ps.pose.orientation.z = qz
                    ps.pose.orientation.w = qw
                    self.gt_path.poses.append(ps)
                    self.gt_path.header.stamp = stamp
                    self.gt_path_pub.publish(self.gt_path)

                # Publish current pose
                pose_msg = PoseStamped()
                pose_msg.header.stamp = stamp
                pose_msg.header.frame_id = self.gt_frame
                pose_msg.pose.position.x = x
                pose_msg.pose.position.y = y
                pose_msg.pose.position.z = z
                pose_msg.pose.orientation.x = qx
                pose_msg.pose.orientation.y = qy
                pose_msg.pose.orientation.z = qz
                pose_msg.pose.orientation.w = qw
                self.gt_pose_pub.publish(pose_msg)

                # Publish arrow marker
                self._publish_arrow_marker(
                    self.gt_marker_pub, x, y, z, qx, qy, qz, qw,
                    stamp, self.gt_frame, marker_id=0,
                    r=0.0, g=1.0, b=0.0, a=1.0,
                    scale_x=0.3, scale_y=0.06, scale_z=0.06
                )

                # Publish error if we have both
                self._publish_error()
                break

    def gt_callback_generic(self, msg):
        """Fallback if phasespace_msgs is not installed - deserialize manually."""
        rospy.logwarn_once("Using generic deserialization for /phasespace/rigids. "
                          "Install phasespace_msgs for best results.")

    # -------------------------------------------------------------------------
    # Odometry Callback
    # -------------------------------------------------------------------------
    def odom_callback(self, msg):
        """Process nav_msgs/Odometry message."""
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        stamp = msg.header.stamp

        self.last_odom_pose = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)

        # Store first odom pose for alignment
        if self.first_odom_pose is None:
            self.first_odom_pose = self.last_odom_pose
            rospy.loginfo("First odom pose captured: (%.3f, %.3f, %.3f)", p.x, p.y, p.z)
            self._try_compute_alignment()

        # Downsample
        self.odom_msg_count += 1
        if self.odom_msg_count % self.path_downsample == 0:
            ps = PoseStamped()
            ps.header.stamp = stamp
            # Publish in GT frame after alignment so both paths overlap in RViz
            ps.header.frame_id = self.gt_frame
            ox, oy, oz, oqx, oqy, oqz, oqw = self._transform_odom_to_gt(
                p.x, p.y, p.z, q.x, q.y, q.z, q.w
            )
            ps.pose.position.x = ox
            ps.pose.position.y = oy
            ps.pose.position.z = oz
            ps.pose.orientation.x = oqx
            ps.pose.orientation.y = oqy
            ps.pose.orientation.z = oqz
            ps.pose.orientation.w = oqw
            self.odom_path.poses.append(ps)
            self.odom_path.header.stamp = stamp
            self.odom_path.header.frame_id = self.gt_frame
            self.odom_path_pub.publish(self.odom_path)

        # Publish arrow marker for odom (in gt_frame so they can be compared)
        ox, oy, oz, oqx, oqy, oqz, oqw = self._transform_odom_to_gt(
            p.x, p.y, p.z, q.x, q.y, q.z, q.w
        )
        self._publish_arrow_marker(
            self.odom_marker_pub, ox, oy, oz, oqx, oqy, oqz, oqw,
            stamp, self.gt_frame, marker_id=1,
            r=1.0, g=0.0, b=0.0, a=1.0,
            scale_x=0.3, scale_y=0.06, scale_z=0.06
        )

        self._publish_error()

    # -------------------------------------------------------------------------
    # Alignment: compute map->odom transform from first matched poses
    # -------------------------------------------------------------------------
    def _try_compute_alignment(self):
        if self.alignment_computed or not self.auto_align:
            return
        if self.first_gt_pose is None or self.first_odom_pose is None:
            return

        gx, gy, gz, gqx, gqy, gqz, gqw = self.first_gt_pose
        ox, oy, oz, oqx, oqy, oqz, oqw = self.first_odom_pose

        # T_map_gt = pose of robot in map frame (from mocap)
        T_gt = tft.quaternion_matrix([gqx, gqy, gqz, gqw])
        T_gt[0, 3] = gx
        T_gt[1, 3] = gy
        T_gt[2, 3] = gz

        # T_odom_robot = pose of robot in odom frame
        T_odom = tft.quaternion_matrix([oqx, oqy, oqz, oqw])
        T_odom[0, 3] = ox
        T_odom[1, 3] = oy
        T_odom[2, 3] = oz

        # alignment: T_map_odom = T_gt * T_odom^-1
        # so that T_map_robot = T_map_odom * T_odom_robot = T_gt at t=0
        T_odom_inv = np.linalg.inv(T_odom)
        self.alignment_tf = T_gt.dot(T_odom_inv)
        self.alignment_computed = True

        q_align = tft.quaternion_from_matrix(self.alignment_tf)
        rospy.loginfo("Alignment computed (map->odom):")
        rospy.loginfo("  Translation: (%.3f, %.3f, %.3f)",
                      self.alignment_tf[0, 3], self.alignment_tf[1, 3], self.alignment_tf[2, 3])
        rospy.loginfo("  Rotation (quat): (%.4f, %.4f, %.4f, %.4f)",
                      q_align[0], q_align[1], q_align[2], q_align[3])

    def _transform_odom_to_gt(self, x, y, z, qx, qy, qz, qw):
        """Transform an odom-frame pose into the GT (map) frame."""
        if not self.alignment_computed:
            return x, y, z, qx, qy, qz, qw

        T_odom = tft.quaternion_matrix([qx, qy, qz, qw])
        T_odom[0, 3] = x
        T_odom[1, 3] = y
        T_odom[2, 3] = z

        T_map = self.alignment_tf.dot(T_odom)
        q = tft.quaternion_from_matrix(T_map)
        return T_map[0, 3], T_map[1, 3], T_map[2, 3], q[0], q[1], q[2], q[3]

    # -------------------------------------------------------------------------
    # Publish alignment TF so RViz and other nodes can use it
    # -------------------------------------------------------------------------
    def publish_alignment_tf(self, event):
        if not self.alignment_computed:
            return

        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = self.gt_frame
        t.child_frame_id = self.odom_frame
        q = tft.quaternion_from_matrix(self.alignment_tf)
        t.transform.translation.x = self.alignment_tf[0, 3]
        t.transform.translation.y = self.alignment_tf[1, 3]
        t.transform.translation.z = self.alignment_tf[2, 3]
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)

    # -------------------------------------------------------------------------
    # Error visualization
    # -------------------------------------------------------------------------
    def _publish_error(self):
        if self.last_gt_pose is None or self.last_odom_pose is None:
            return

        gx, gy, gz = self.last_gt_pose[0], self.last_gt_pose[1], self.last_gt_pose[2]

        # Transform odom pose into GT frame for fair comparison
        ox, oy, oz, _, _, _, _ = self._transform_odom_to_gt(*self.last_odom_pose)

        dx = gx - ox
        dy = gy - oy
        dz = gz - oz
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)

        stamp = rospy.Time.now()

        # Yellow line between GT and odom positions
        line = Marker()
        line.header.frame_id = self.gt_frame
        line.header.stamp = stamp
        line.ns = "error_line"
        line.id = 10
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale.x = 0.02
        line.color = ColorRGBA(1.0, 1.0, 0.0, 0.8)
        line.points.append(Point(gx, gy, gz))
        line.points.append(Point(ox, oy, oz))
        line.lifetime = rospy.Duration(0.3)
        self.error_marker_pub.publish(line)

        # Text showing distance error
        text = Marker()
        text.header.frame_id = self.gt_frame
        text.header.stamp = stamp
        text.ns = "error_text"
        text.id = 11
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = (gx + ox) / 2.0
        text.pose.position.y = (gy + oy) / 2.0
        text.pose.position.z = (gz + oz) / 2.0 + 0.3
        text.scale.z = 0.12
        text.color = ColorRGBA(1.0, 1.0, 1.0, 1.0)
        text.text = "Error: %.3f m" % dist
        text.lifetime = rospy.Duration(0.3)
        self.error_text_pub.publish(text)

    # -------------------------------------------------------------------------
    # Marker helpers
    # -------------------------------------------------------------------------
    def _publish_arrow_marker(self, pub, x, y, z, qx, qy, qz, qw,
                              stamp, frame_id, marker_id,
                              r, g, b, a, scale_x, scale_y, scale_z):
        m = Marker()
        m.header.frame_id = frame_id
        m.header.stamp = stamp
        m.ns = "pose_arrow"
        m.id = marker_id
        m.type = Marker.ARROW
        m.action = Marker.ADD
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = z
        m.pose.orientation.x = qx
        m.pose.orientation.y = qy
        m.pose.orientation.z = qz
        m.pose.orientation.w = qw
        m.scale.x = scale_x
        m.scale.y = scale_y
        m.scale.z = scale_z
        m.color = ColorRGBA(r, g, b, a)
        m.lifetime = rospy.Duration(0.2)
        pub.publish(m)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        node = GTComparisonNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
