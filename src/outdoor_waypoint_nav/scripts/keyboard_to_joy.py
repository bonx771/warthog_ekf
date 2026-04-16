#!/usr/bin/env python3
import sys
import termios
import tty
import textwrap
import signal

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy


class KeyboardToJoy:
    def __init__(self):
        self.output_topic = rospy.get_param("~output_topic", "/joy_teleop/joy")
        self.cmd_topic = rospy.get_param("~cmd_topic", "/joy_teleop/cmd_vel")
        self.button_count = rospy.get_param("~button_count", 8)
        self.press_repeats = rospy.get_param("~press_repeats", 5)
        self.release_repeats = rospy.get_param("~release_repeats", 2)
        self.repeat_delay = rospy.get_param("~repeat_delay", 0.03)
        self.linear_speed = rospy.get_param("~linear_speed", 0.8)
        self.angular_speed = rospy.get_param("~angular_speed", 0.8)
        self.publish_rate = rospy.get_param("~publish_rate", 10.0)

        self.key_to_button = {
            "l": 4,  # LB
            "r": 5,  # RB
            "s": 7,  # START
            "b": 1,  # B
            "y": 3,  # Y
            "k": 6,  # BACK
        }

        self.pub = rospy.Publisher(self.output_topic, Joy, queue_size=10)
        self.cmd_pub = rospy.Publisher(self.cmd_topic, Twist, queue_size=10)
        self._running = True
        self.current_twist = Twist()
        self.manual_active = False

        signal.signal(signal.SIGINT, self._handle_sigint)
        rospy.Timer(rospy.Duration(1.0 / self.publish_rate), self._publish_current_twist)

    def _handle_sigint(self, _signum, _frame):
        self._stop_motion()
        self._running = False
        rospy.signal_shutdown("Ctrl-C pressed")

    def _publish_current_twist(self, _event):
        if self.manual_active:
            self.cmd_pub.publish(self.current_twist)

    def _set_motion(self, linear=0.0, angular=0.0):
        self.current_twist.linear.x = linear
        self.current_twist.angular.z = angular
        self.manual_active = True
        self.cmd_pub.publish(self.current_twist)

    def _stop_motion(self):
        self.current_twist.linear.x = 0.0
        self.current_twist.angular.z = 0.0
        self.cmd_pub.publish(self.current_twist)
        self.manual_active = False

    def _publish_button(self, button_index):
        pressed_msg = Joy()
        pressed_msg.axes = []
        pressed_msg.buttons = [0] * self.button_count
        pressed_msg.buttons[button_index] = 1

        released_msg = Joy()
        released_msg.axes = []
        released_msg.buttons = [0] * self.button_count

        for _ in range(self.press_repeats):
            pressed_msg.header.stamp = rospy.Time.now()
            self.pub.publish(pressed_msg)
            rospy.sleep(self.repeat_delay)

        for _ in range(self.release_repeats):
            released_msg.header.stamp = rospy.Time.now()
            self.pub.publish(released_msg)
            rospy.sleep(self.repeat_delay)

    def _getch(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            char = sys.stdin.read(1)
            # In raw mode Ctrl-C arrives as '\x03' instead of raising KeyboardInterrupt.
            if char == "\x03":
                self._handle_sigint(None, None)
                return None
            return char
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def run(self):
        banner = textwrap.dedent(
            """
            ==================== Keyboard To Joy ====================
            w -> drive forward
            x -> drive backward
            a -> turn left
            d -> turn right
            space -> stop motion
            l -> LB     start waypoint collection
            r -> RB     start waypoint following
            s -> START  heading calibration
            b -> B      stop robot motion
            y -> Y      continue or collect waypoint
            k -> BACK   end waypoint collection
            q -> quit
            ========================================================
            """
        ).strip("\n")
        sys.stdout.write("\n" + banner + "\n\n")
        sys.stdout.flush()

        while not rospy.is_shutdown() and self._running:
            key = self._getch()
            if key is None:
                continue
            key = key.lower()
            if key == "q":
                break
            if key == "w":
                self._set_motion(self.linear_speed, 0.0)
                rospy.loginfo("Keyboard drive: forward")
                continue
            if key == "x":
                self._set_motion(-self.linear_speed, 0.0)
                rospy.loginfo("Keyboard drive: backward")
                continue
            if key == "a":
                self._set_motion(0.0, self.angular_speed)
                rospy.loginfo("Keyboard drive: turn left")
                continue
            if key == "d":
                self._set_motion(0.0, -self.angular_speed)
                rospy.loginfo("Keyboard drive: turn right")
                continue
            if key == " ":
                self._stop_motion()
                rospy.loginfo("Keyboard drive: stop")
                continue
            if key in self.key_to_button:
                rospy.loginfo("Sent Joy button for key: %s", key)
                self._publish_button(self.key_to_button[key])

        self._stop_motion()


if __name__ == "__main__":
    rospy.init_node("keyboard_to_joy")
    KeyboardToJoy().run()
