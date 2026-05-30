#!/usr/bin/env python
from __future__ import print_function

import json
import math
import socket
import time

import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
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


def _param(name, default):
    return rospy.get_param("~" + name, rospy.get_param(name, default))


def yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def angle_delta(target, current):
    return math.atan2(math.sin(target - current), math.cos(target - current))


def clamp(value, low, high):
    return max(low, min(high, value))


def sign_or_zero(value):
    if value > 0.0:
        return 1.0
    if value < 0.0:
        return -1.0
    return 0.0


def _cfg(cfg, defaults, name, fallback=None):
    if name in cfg:
        return cfg[name]
    if name in defaults:
        return defaults[name]
    return fallback


class MocapShuttleRobot(object):
    def __init__(self, node, cfg, index):
        self.node = node
        self.cfg = cfg
        self.index = index
        self.robot_id = str(cfg.get("robot_id", "robot_%02d" % (index + 1)))
        self.host = str(cfg.get("host", "")).strip()
        self.port = int(cfg.get("port", node.default_port))
        self.enabled = _as_bool(cfg.get("enabled", bool(self.host)))

        self.distance = max(0.0, float(_cfg(cfg, node.defaults, "distance", 1.0)))
        self.out_target = self.distance + float(_cfg(cfg, node.defaults, "out_target_offset", 0.0))
        self.back_target = float(_cfg(cfg, node.defaults, "back_target_offset", 0.0))
        self.cycles = max(1, int(_cfg(cfg, node.defaults, "cycles", 10)))
        self.max_speed = abs(float(_cfg(cfg, node.defaults, "max_speed", 0.14)))
        self.min_speed = min(abs(float(_cfg(cfg, node.defaults, "min_speed", 0.025))), self.max_speed)
        self.along_kp = max(0.0, float(_cfg(cfg, node.defaults, "along_kp", 0.45)))
        self.cross_heading_kp = max(0.0, float(_cfg(cfg, node.defaults, "cross_heading_kp", 0.60)))
        self.heading_kp = max(0.0, float(_cfg(cfg, node.defaults, "heading_kp", 0.70)))
        self.max_angular = abs(float(_cfg(cfg, node.defaults, "max_angular", 0.18)))
        self.max_lateral = abs(float(_cfg(cfg, node.defaults, "max_lateral", 0.0)))
        self.cross_track_kp = max(0.0, float(_cfg(cfg, node.defaults, "cross_track_kp", 0.0)))
        self.enable_lateral = _as_bool(_cfg(cfg, node.defaults, "enable_lateral", False))
        self.distance_tolerance = max(0.0, float(_cfg(cfg, node.defaults, "distance_tolerance", 0.04)))
        self.min_speed_distance = max(self.distance_tolerance, float(_cfg(cfg, node.defaults, "min_speed_distance", 0.08)))
        self.cross_track_tolerance = max(0.0, float(_cfg(cfg, node.defaults, "cross_track_tolerance", 0.12)))
        self.max_cross_track = max(self.cross_track_tolerance, float(_cfg(cfg, node.defaults, "max_cross_track", 0.75)))
        self.heading_tolerance = math.radians(max(0.0, float(_cfg(cfg, node.defaults, "heading_tolerance_deg", 12.0))))
        self.max_heading_error = math.radians(max(0.0, float(_cfg(cfg, node.defaults, "max_heading_error_deg", 90.0))))
        self.heading_gate = math.radians(max(0.0, float(_cfg(cfg, node.defaults, "heading_gate_deg", 60.0))))
        self.max_cross_heading_offset = math.radians(max(0.0, float(_cfg(cfg, node.defaults, "max_cross_heading_offset_deg", 15.0))))
        self.endpoint_requires_cross_track = _as_bool(_cfg(cfg, node.defaults, "endpoint_requires_cross_track", False))
        self.endpoint_requires_heading = _as_bool(_cfg(cfg, node.defaults, "endpoint_requires_heading", False))
        self.settle_time = max(0.0, float(_cfg(cfg, node.defaults, "settle_time", 0.25)))
        self.pause = max(0.0, float(_cfg(cfg, node.defaults, "pause", 1.0)))
        self.odom_timeout = max(0.05, float(_cfg(cfg, node.defaults, "odom_timeout", 0.25)))
        self.mocap_lost_timeout = max(
            self.odom_timeout,
            float(_cfg(cfg, node.defaults, "mocap_lost_timeout", 5.0)),
        )
        self.abort_on_mocap_loss = _as_bool(_cfg(cfg, node.defaults, "abort_on_mocap_loss", False))
        self.leg_timeout = max(1.0, float(_cfg(cfg, node.defaults, "leg_timeout", 40.0)))
        self.stuck_timeout = max(1.0, float(_cfg(cfg, node.defaults, "stuck_timeout", 10.0)))
        self.progress_epsilon = max(0.0, float(_cfg(cfg, node.defaults, "progress_epsilon", 0.01)))
        self.start_time = node.base_start_time + index * node.start_stagger + float(cfg.get("start_delay", 0.0))

        self.pose_topic = str(cfg.get("pose_topic", "/gt/%s/pose" % self.robot_id))
        self.lane_yaw_deg = cfg.get("lane_yaw_deg", None)

        self.pose = None
        self.last_pose_wall = 0.0
        self.origin = None
        self.lane_yaw = None
        self.target = self.out_target
        self.leg_index = 1
        self.state = "disabled" if not self.enabled else "waiting_pose"
        self.pause_until = None
        self.settle_start = None
        self.leg_start = None
        self.best_abs_along_error = None
        self.last_progress_time = None
        self.seq = 0

        self.cmd_pub = rospy.Publisher("/mocap_control/%s/cmd_vel" % self.robot_id, TwistStamped, queue_size=5)
        self.status_pub = rospy.Publisher("/mocap_control/%s/status" % self.robot_id, String, queue_size=5)
        if self.pose_topic.endswith("/odom"):
            self.sub = rospy.Subscriber(self.pose_topic, Odometry, self._odom_cb, queue_size=5)
        else:
            self.sub = rospy.Subscriber(self.pose_topic, PoseStamped, self._pose_cb, queue_size=5)

        if self.enabled:
            rospy.loginfo("Mocap controller robot %s -> %s:%d topic=%s", self.robot_id, self.host, self.port, self.pose_topic)
        else:
            rospy.logwarn("Mocap controller robot %s disabled; host is empty or enabled=false", self.robot_id)

    def _odom_cb(self, msg):
        self._update_pose(msg.pose.pose)

    def _pose_cb(self, msg):
        self._update_pose(msg.pose)

    def _update_pose(self, pose_msg):
        p = pose_msg.position
        self.pose = (p.x, p.y, yaw_from_quat(pose_msg.orientation))
        self.last_pose_wall = time.time()

    def _lane_state(self):
        x, y, yaw = self.pose
        dx = x - self.origin[0]
        dy = y - self.origin[1]
        forward_x = math.cos(self.lane_yaw)
        forward_y = math.sin(self.lane_yaw)
        left_x = -forward_y
        left_y = forward_x
        along = dx * forward_x + dy * forward_y
        cross = dx * left_x + dy * left_y
        yaw_error = angle_delta(self.lane_yaw, yaw)
        return along, cross, yaw, yaw_error

    def _start_run(self, now):
        x, y, yaw = self.pose
        self.origin = (x, y)
        if self.lane_yaw_deg is None:
            self.lane_yaw = yaw
        else:
            self.lane_yaw = math.radians(float(self.lane_yaw_deg))
        self.target = self.out_target
        self.leg_index = 1
        self.leg_start = now
        self.best_abs_along_error = None
        self.last_progress_time = now
        self.state = "running"
        rospy.loginfo("%s started: origin=(%.3f, %.3f) lane_yaw=%.1fdeg out=%.3f back=%.3f",
                      self.robot_id, x, y, math.degrees(self.lane_yaw), self.out_target, self.back_target)

    def _advance_leg(self, now):
        if self.leg_index >= self.cycles * 2:
            self.state = "done"
            self._send_command(0.0, 0.0, 0.0)
            rospy.loginfo("%s complete after %d legs", self.robot_id, self.leg_index)
            return

        self.leg_index += 1
        self.target = self.back_target if self.leg_index % 2 == 0 else self.out_target
        self.pause_until = now + self.pause
        self.state = "pause"
        self.settle_start = None
        self.best_abs_along_error = None
        self.last_progress_time = now
        self._send_command(0.0, 0.0, 0.0)

    def _command_for_error(self, along_error, cross, yaw, now):
        along_cmd = clamp(self.along_kp * along_error, -self.max_speed, self.max_speed)
        if abs(along_error) > self.min_speed_distance and abs(along_cmd) < self.min_speed:
            along_cmd = sign_or_zero(along_error) * self.min_speed

        direction = sign_or_zero(along_error)
        if direction == 0.0:
            direction = 1.0

        cross_heading = -direction * clamp(
            self.cross_heading_kp * cross,
            -self.max_cross_heading_offset,
            self.max_cross_heading_offset,
        )
        target_yaw = self.lane_yaw + cross_heading
        target_yaw_error = angle_delta(target_yaw, yaw)

        linear_x = along_cmd
        linear_y = 0.0
        if self.enable_lateral:
            linear_y = clamp(-self.cross_track_kp * cross, -self.max_lateral, self.max_lateral)

        if abs(target_yaw_error) > self.heading_gate:
            linear_x = 0.0
            linear_y = 0.0

        angular_z = clamp(self.heading_kp * target_yaw_error, -self.max_angular, self.max_angular)
        return linear_x, linear_y, angular_z, target_yaw_error

    def _send_command(self, linear_x, linear_y, angular_z):
        if not self.enabled:
            return
        self.seq += 1
        packet = {
            "robot_id": self.robot_id,
            "seq": self.seq,
            "stamp": time.time(),
            "linear_x": float(linear_x),
            "linear_y": float(linear_y),
            "angular_z": float(angular_z),
        }
        try:
            self.node.sock.sendto(json.dumps(packet, sort_keys=True).encode("utf-8"), (self.host, self.port))
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "%s failed to send UDP command: %s", self.robot_id, exc)

        msg = TwistStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.robot_id
        msg.twist.linear.x = linear_x
        msg.twist.linear.y = linear_y
        msg.twist.angular.z = angular_z
        self.cmd_pub.publish(msg)

    def _publish_status(self, extra=None):
        status = {
            "robot_id": self.robot_id,
            "state": self.state,
            "host": self.host,
            "port": self.port,
            "leg": self.leg_index,
            "target": round(self.target, 4),
        }
        if extra:
            status.update(extra)
        self.status_pub.publish(json.dumps(status, sort_keys=True))

    def update(self, now):
        if not self.enabled:
            self._publish_status()
            return True

        if self.pose is None:
            self._send_command(0.0, 0.0, 0.0)
            self.state = "waiting_pose"
            self._publish_status()
            return False

        if now < self.start_time:
            self._send_command(0.0, 0.0, 0.0)
            self.state = "waiting_start"
            self._publish_status({"start_in_s": round(self.start_time - now, 2)})
            return False

        stale_s = now - self.last_pose_wall
        if self.state in ["waiting_pose", "waiting_start"]:
            if stale_s > self.odom_timeout:
                self._send_command(0.0, 0.0, 0.0)
                self.state = "waiting_fresh_mocap"
                rospy.logwarn_throttle(2.0, "%s waiting: stale mocap pose for %.2fs", self.robot_id, stale_s)
                self._publish_status({"stale_pose_s": round(stale_s, 2)})
                return False
            self._start_run(now)
        elif self.state == "waiting_fresh_mocap":
            if stale_s > self.odom_timeout:
                self._send_command(0.0, 0.0, 0.0)
                rospy.logwarn_throttle(2.0, "%s waiting: stale mocap pose for %.2fs", self.robot_id, stale_s)
                self._publish_status({"stale_pose_s": round(stale_s, 2)})
                return False
            self._start_run(now)

        if self.state == "done":
            self._send_command(0.0, 0.0, 0.0)
            self._publish_status()
            return True
        if self.state == "aborted":
            self._send_command(0.0, 0.0, 0.0)
            self._publish_status()
            return True
        if self.state == "pause":
            self._send_command(0.0, 0.0, 0.0)
            self._publish_status({"pause_remaining_s": round(max(0.0, self.pause_until - now), 2)})
            if now >= self.pause_until:
                self.state = "running"
                self.leg_start = now
                self.best_abs_along_error = None
                self.last_progress_time = now
            return False

        if stale_s > self.mocap_lost_timeout:
            self._send_command(0.0, 0.0, 0.0)
            if self.abort_on_mocap_loss:
                self.state = "aborted"
                rospy.logerr("%s abort: lost mocap pose for %.2fs", self.robot_id, stale_s)
                return True
            self.state = "holding_lost_mocap"
            rospy.logerr_throttle(5.0, "%s hold: lost mocap pose for %.2fs", self.robot_id, stale_s)
            self._publish_status({"stale_pose_s": round(stale_s, 2)})
            return False
        if stale_s > self.odom_timeout:
            self._send_command(0.0, 0.0, 0.0)
            rospy.logwarn_throttle(2.0, "%s hold: stale mocap pose for %.2fs", self.robot_id, stale_s)
            self._publish_status({"stale_pose_s": round(stale_s, 2)})
            return False
        if self.state == "holding_lost_mocap":
            rospy.loginfo("%s recovered fresh mocap pose; resuming leg %d", self.robot_id, self.leg_index)
            self.state = "running"
            self.last_progress_time = now

        along, cross, yaw, yaw_error = self._lane_state()
        along_error = self.target - along
        abs_along_error = abs(along_error)

        if self.best_abs_along_error is None or abs_along_error < self.best_abs_along_error - self.progress_epsilon:
            self.best_abs_along_error = abs_along_error
            self.last_progress_time = now

        cross_ok = (not self.endpoint_requires_cross_track) or abs(cross) <= self.cross_track_tolerance
        heading_ok = (not self.endpoint_requires_heading) or abs(yaw_error) <= self.heading_tolerance
        position_ok = abs_along_error <= self.distance_tolerance and cross_ok and heading_ok

        if position_ok:
            self._send_command(0.0, 0.0, 0.0)
            if self.settle_start is None:
                self.settle_start = now
            if now - self.settle_start >= self.settle_time:
                rospy.loginfo("%s done leg=%d target=%.3f along=%.3f cross=%.3f yaw=%.1fdeg",
                              self.robot_id, self.leg_index, self.target, along, cross, math.degrees(yaw_error))
                self._advance_leg(now)
            self._publish_status({
                "along": round(along, 4),
                "cross": round(cross, 4),
                "yaw_error_deg": round(math.degrees(yaw_error), 2),
            })
            return False
        self.settle_start = None

        if abs(cross) > self.max_cross_track:
            self.state = "aborted"
            self._send_command(0.0, 0.0, 0.0)
            rospy.logerr("%s abort: cross-track %.3fm exceeded %.3fm", self.robot_id, cross, self.max_cross_track)
            return True
        if abs(yaw_error) > self.max_heading_error:
            self.state = "aborted"
            self._send_command(0.0, 0.0, 0.0)
            rospy.logerr("%s abort: yaw %.1fdeg exceeded %.1fdeg",
                         self.robot_id, math.degrees(yaw_error), math.degrees(self.max_heading_error))
            return True
        if now - self.leg_start > self.leg_timeout:
            self.state = "aborted"
            self._send_command(0.0, 0.0, 0.0)
            rospy.logerr("%s abort: leg timeout %.1fs", self.robot_id, self.leg_timeout)
            return True
        if now - self.last_progress_time > self.stuck_timeout:
            self.state = "aborted"
            self._send_command(0.0, 0.0, 0.0)
            rospy.logerr("%s abort: no along-track progress for %.1fs", self.robot_id, self.stuck_timeout)
            return True

        linear_x, linear_y, angular_z, target_yaw_error = self._command_for_error(along_error, cross, yaw, now)
        self._send_command(linear_x, linear_y, angular_z)
        self._publish_status({
            "along": round(along, 4),
            "along_error": round(along_error, 4),
            "cross": round(cross, 4),
            "yaw_error_deg": round(math.degrees(yaw_error), 2),
            "target_yaw_error_deg": round(math.degrees(target_yaw_error), 2),
            "cmd": [round(linear_x, 4), round(linear_y, 4), round(angular_z, 4)],
        })
        return False

    def stop(self):
        self._send_command(0.0, 0.0, 0.0)


class MocapGuidedShuttleController(object):
    def __init__(self):
        rospy.init_node("mocap_guided_shuttle_controller")

        self.default_port = int(_param("default_port", 11500))
        self.command_rate = max(5.0, float(_param("command_rate", 20.0)))
        self.start_stagger = max(0.0, float(_param("start_stagger", 0.0)))
        self.shutdown_when_done = _as_bool(_param("shutdown_when_done", True))

        start_at_epoch = float(_param("start_at_epoch", 0.0))
        start_delay = max(0.0, float(_param("start_delay", 0.0)))
        if start_at_epoch > 0.0:
            self.base_start_time = start_at_epoch + start_delay
        else:
            self.base_start_time = time.time() + start_delay

        self.defaults = dict(_param("defaults", {}))
        robots_cfg = _param("robots", [])
        if not robots_cfg:
            raise RuntimeError("No robots configured for mocap-guided shuttle")

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.robots = [MocapShuttleRobot(self, cfg, index) for index, cfg in enumerate(robots_cfg)]

        rospy.loginfo("Mocap-guided shuttle controller started")
        rospy.loginfo("  robots=%s", ", ".join(robot.robot_id for robot in self.robots if robot.enabled))
        rospy.loginfo("  base_start_epoch=%.3f command_rate=%.1fHz", self.base_start_time, self.command_rate)

    def stop_all(self, repeats=8):
        rate = rospy.Rate(20)
        for _ in range(repeats):
            for robot in self.robots:
                robot.stop()
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                break

    def run(self):
        rospy.on_shutdown(self.stop_all)
        rate = rospy.Rate(self.command_rate)
        while not rospy.is_shutdown():
            now = time.time()
            done = [robot.update(now) for robot in self.robots if robot.enabled]
            if done and all(done) and self.shutdown_when_done:
                rospy.loginfo("All enabled mocap-guided shuttle robots are finished")
                self.stop_all()
                return
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                break


if __name__ == "__main__":
    MocapGuidedShuttleController().run()
