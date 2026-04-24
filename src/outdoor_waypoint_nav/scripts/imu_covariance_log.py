#!/usr/bin/env python3

import math

import rospy
from sensor_msgs.msg import Imu


def _yaw_from_quaternion(quaternion_msg):
    return math.atan2(
        2.0 * (quaternion_msg.w * quaternion_msg.z + quaternion_msg.x * quaternion_msg.y),
        1.0 - 2.0 * (quaternion_msg.y * quaternion_msg.y + quaternion_msg.z * quaternion_msg.z),
    )


def _format_values(values):
    return "[" + ", ".join("{:.6g}".format(value) for value in values) + "]"


class ImuCovarianceLog:
    def __init__(self):
        self.log_rate_hz = rospy.get_param("~log_rate_hz", 10.0)
        self.last_log_time = rospy.Time(0)
        self.min_log_period = rospy.Duration(0.0)
        if self.log_rate_hz > 0.0:
            self.min_log_period = rospy.Duration(1.0 / self.log_rate_hz)

        self.subscriber = rospy.Subscriber(
            "imu/data",
            Imu,
            self._callback,
            queue_size=20,
            tcp_nodelay=True,
        )

    def _callback(self, msg):
        now = rospy.Time.now()
        if (
            self.min_log_period.to_sec() > 0.0
            and now - self.last_log_time < self.min_log_period
        ):
            return

        yaw_deg = math.degrees(_yaw_from_quaternion(msg.orientation))
        rospy.loginfo(
            "imu_yaw = %.2f deg, orient_covariance = %s, angular_vel_covariance = %s, linear_acc_covariance = %s",
            yaw_deg,
            _format_values(msg.orientation_covariance),
            _format_values(msg.angular_velocity_covariance),
            _format_values(msg.linear_acceleration_covariance),
        )
        self.last_log_time = now


if __name__ == "__main__":
    rospy.init_node("imu_covariance_log")
    ImuCovarianceLog()
    rospy.spin()
