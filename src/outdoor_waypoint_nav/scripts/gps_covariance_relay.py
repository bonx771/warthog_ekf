#!/usr/bin/env python3

import copy

import rospy
from sensor_msgs.msg import NavSatFix


# Edit these defaults if you want one central place to control all GPS covariances.
DEFAULT_POSITION_DIAG = (0.1, 0.1, 0.1)


def _read_diag_param(name, default_diag):
    values = rospy.get_param("~" + name, list(default_diag))
    if not isinstance(values, (list, tuple)) or len(values) != 3:
        raise rospy.ROSException(
            "~{} must be a list of three numbers, got {!r}".format(name, values)
        )

    diag = tuple(float(value) for value in values)
    if any(value < 0.0 for value in diag):
        raise rospy.ROSException(
            "~{} must contain non-negative values, got {!r}".format(name, values)
        )
    return diag


def _diag_to_covariance(diag):
    return [
        diag[0], 0.0, 0.0,
        0.0, diag[1], 0.0,
        0.0, 0.0, diag[2],
    ]


class GpsCovarianceRelay:
    def __init__(self):
        input_topic = rospy.get_param("~input_topic", "/gps/fix")
        output_topic = rospy.get_param("~output_topic", "/outdoor_waypoint_nav/gps/fix_with_covariance")
        position_diag = _read_diag_param("position_diag", DEFAULT_POSITION_DIAG)
        self.position_covariance = _diag_to_covariance(position_diag)

        self.publisher = rospy.Publisher(output_topic, NavSatFix, queue_size=10)
        self.subscriber = rospy.Subscriber(input_topic, NavSatFix, self._callback, queue_size=10)

        rospy.loginfo(
            "Relaying GPS from %s to %s with hard covariance override: position diag %s",
            input_topic,
            output_topic,
            position_diag,
        )

    def _callback(self, msg):
        relay_msg = copy.deepcopy(msg)
        relay_msg.position_covariance = list(self.position_covariance)
        relay_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self.publisher.publish(relay_msg)


if __name__ == "__main__":
    rospy.init_node("gps_covariance_relay")
    GpsCovarianceRelay()
    rospy.spin()
