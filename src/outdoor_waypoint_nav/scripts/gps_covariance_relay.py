#!/usr/bin/env python3

import copy

import rospy
from sensor_msgs.msg import NavSatFix


class GpsCovarianceRelay:
    def __init__(self):
        input_topic = rospy.get_param("~input_topic", "/gps/fix")
        output_topic = rospy.get_param("~output_topic", "/outdoor_waypoint_nav/gps/fix_with_covariance")
        self.xy_variance = float(rospy.get_param("~xy_variance", 1.0))
        self.z_variance = float(rospy.get_param("~z_variance", 4.0))
        self.override_zero_only = bool(rospy.get_param("~override_zero_only", True))

        self.publisher = rospy.Publisher(output_topic, NavSatFix, queue_size=10)
        self.subscriber = rospy.Subscriber(input_topic, NavSatFix, self._callback, queue_size=10)

        rospy.loginfo(
            "Relaying GPS from %s to %s with covariance diag [%.3f, %.3f, %.3f]",
            input_topic,
            output_topic,
            self.xy_variance,
            self.xy_variance,
            self.z_variance,
        )

    @staticmethod
    def _has_known_covariance(msg):
        diag = (
            msg.position_covariance[0],
            msg.position_covariance[4],
            msg.position_covariance[8],
        )
        return (
            msg.position_covariance_type != NavSatFix.COVARIANCE_TYPE_UNKNOWN
            and any(value > 0.0 for value in diag)
        )

    def _callback(self, msg):
        relay_msg = copy.deepcopy(msg)

        if not self.override_zero_only or not self._has_known_covariance(relay_msg):
            relay_msg.position_covariance = [
                self.xy_variance, 0.0, 0.0,
                0.0, self.xy_variance, 0.0,
                0.0, 0.0, self.z_variance,
            ]
            relay_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

        self.publisher.publish(relay_msg)


if __name__ == "__main__":
    rospy.init_node("gps_covariance_relay")
    GpsCovarianceRelay()
    rospy.spin()
