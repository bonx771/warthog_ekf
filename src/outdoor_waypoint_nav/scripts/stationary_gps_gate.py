#!/usr/bin/env python3

import copy
import math

import rospy
from nav_msgs.msg import Odometry


def _covariance_with_stationary_floor(covariance, xy_floor, z_floor):
    cov = list(covariance)
    if len(cov) != 36:
        cov = [0.0] * 36

    cov[0] = max(float(cov[0]), xy_floor)
    cov[7] = max(float(cov[7]), xy_floor)
    cov[14] = max(float(cov[14]), z_floor)
    return cov


class StationaryGpsGate:
    def __init__(self):
        gps_topic = rospy.get_param("~gps_odom_topic", "odometry/gps")
        motion_topic = rospy.get_param("~motion_odom_topic", "odometry/filtered_odom")
        output_topic = rospy.get_param("~output_topic", "odometry/gps_filter")

        self.linear_threshold = float(rospy.get_param("~linear_stop_threshold", 0.03))
        self.angular_threshold = float(rospy.get_param("~angular_stop_threshold", 0.01))
        self.stop_confirm_duration = float(
            rospy.get_param("~stop_confirm_duration", 0.75)
        )
        self.motion_timeout = float(rospy.get_param("~motion_timeout", 0.5))
        self.freeze_position = bool(rospy.get_param("~freeze_stationary_position", True))
        self.stationary_xy_covariance = float(
            rospy.get_param("~stationary_xy_covariance", 1000000.0)
        )
        self.stationary_z_covariance = float(
            rospy.get_param("~stationary_z_covariance", 1000000.0)
        )
        self.copy_motion_orientation = bool(
            rospy.get_param("~copy_motion_orientation", True)
        )

        self.last_motion_time = None
        self.last_linear_speed = float("inf")
        self.last_angular_speed = float("inf")
        self.last_motion_orientation = None
        self.stationary_since = None
        self.was_stationary = False
        self.stationary_anchor = None

        self.publisher = rospy.Publisher(output_topic, Odometry, queue_size=10)
        self.motion_subscriber = rospy.Subscriber(
            motion_topic,
            Odometry,
            self._motion_callback,
            queue_size=20,
            tcp_nodelay=True,
        )
        self.gps_subscriber = rospy.Subscriber(
            gps_topic,
            Odometry,
            self._gps_callback,
            queue_size=20,
            tcp_nodelay=True,
        )

        rospy.loginfo(
            "Stationary GPS gate: gps=%s, motion=%s, output=%s, "
            "linear<=%.3f m/s, angular<=%.3f rad/s, confirm=%.2f s",
            gps_topic,
            motion_topic,
            output_topic,
            self.linear_threshold,
            self.angular_threshold,
            self.stop_confirm_duration,
        )

    def _motion_callback(self, msg):
        now = rospy.Time.now()
        twist = msg.twist.twist
        self.last_motion_time = now
        self.last_linear_speed = math.hypot(twist.linear.x, twist.linear.y)
        self.last_angular_speed = abs(twist.angular.z)
        self.last_motion_orientation = copy.deepcopy(msg.pose.pose.orientation)

        stopped = (
            self.last_linear_speed <= self.linear_threshold
            and self.last_angular_speed <= self.angular_threshold
        )

        if stopped:
            if self.stationary_since is None:
                self.stationary_since = now
        else:
            self.stationary_since = None
            self.stationary_anchor = None

    def _is_stationary(self):
        if self.last_motion_time is None or self.stationary_since is None:
            return False

        now = rospy.Time.now()
        if (now - self.last_motion_time).to_sec() > self.motion_timeout:
            return False

        return (now - self.stationary_since).to_sec() >= self.stop_confirm_duration

    def _apply_motion_orientation(self, msg):
        if self.copy_motion_orientation and self.last_motion_orientation is not None:
            msg.pose.pose.orientation = copy.deepcopy(self.last_motion_orientation)

    def _gps_callback(self, msg):
        stationary = self._is_stationary()

        if not stationary:
            if self.was_stationary:
                rospy.loginfo(
                    "Stationary GPS gate released: speed=%.4f m/s, yaw_rate=%.4f rad/s",
                    self.last_linear_speed,
                    self.last_angular_speed,
                )
            self.was_stationary = False
            self.stationary_anchor = None
            out = copy.deepcopy(msg)
            self._apply_motion_orientation(out)
            self.publisher.publish(out)
            return

        if self.stationary_anchor is None:
            self.stationary_anchor = copy.deepcopy(msg)
            rospy.loginfo(
                "Stationary GPS gate locked at x=%.3f y=%.3f; inflating GPS covariance while stopped",
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
            )

        out = copy.deepcopy(msg)
        if self.freeze_position:
            out.pose.pose.position = copy.deepcopy(
                self.stationary_anchor.pose.pose.position
            )

        self._apply_motion_orientation(out)
        out.pose.covariance = _covariance_with_stationary_floor(
            out.pose.covariance,
            self.stationary_xy_covariance,
            self.stationary_z_covariance,
        )

        self.was_stationary = True
        self.publisher.publish(out)


if __name__ == "__main__":
    rospy.init_node("stationary_gps_gate")
    StationaryGpsGate()
    rospy.spin()
