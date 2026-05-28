#!/usr/bin/env python3

import select
import struct
import termios
import tty

import rospy
import serial
from sensor_msgs.msg import Imu


HEADER = b"\x10\x10"
CHECKSUM_INDEX = 4
OUTPUT_CONTROL_CMD = 0x84
GYROCOMPASS_START_FRAME = bytes.fromhex("10 10 86 09 AF 00 00 00 00")
ALLOWED_STREAM_HZ = (1, 2, 5, 10, 25, 50)


def checksum(frame):
    return sum(byte for i, byte in enumerate(frame) if i != CHECKSUM_INDEX) & 0xFF


def hex_bytes(data):
    return " ".join(f"{byte:02X}" for byte in data)


def output_control_frame(command, hz, data_mask):
    payload = struct.pack(">BHI", command, hz, data_mask)
    frame = bytearray(HEADER + bytes([OUTPUT_CONTROL_CMD, 12, 0]) + payload)
    frame[CHECKSUM_INDEX] = checksum(frame)
    return bytes(frame)


class ImuCalibrationNode:
    def __init__(self):
        self.port = rospy.get_param("~serial_port", "/dev/ttyUSB0")
        self.baud = int(rospy.get_param("~baud_rate", 115200))
        self.timeout = float(rospy.get_param("~serial_timeout_sec", 0.05))
        self.delay = float(rospy.get_param("~startup_command_delay_sec", 0.05))
        self.stream_hz = int(rospy.get_param("~stream_output_hz", 50))
        self.stream_data_mask = int(rospy.get_param("~stream_data_mask", 1))
        self.send_stop_before_stream = bool(rospy.get_param("~send_stop_before_stream", True))
        self.send_gyrocompass_start = bool(rospy.get_param("~send_gyrocompass_start", True))
        self.calibration_duration = float(rospy.get_param("~calibration_duration_sec", 480.0))
        self.ready_timeout = float(rospy.get_param("~ready_check_timeout_sec", 10.0))
        self.auto_start = bool(rospy.get_param("~auto_start", True))
        self.keyboard_start = bool(rospy.get_param("~keyboard_start_enabled", False))
        self.imu_topic = rospy.get_param("~imu_topic", "/imu/data_new")

        self.serial = None
        self.tty = None
        self.tty_settings = None
        self.started = False
        self.stream_sent = False
        self.ready_reported = False
        self.start_time = None
        self.msg_count = 0
        self.msg_count_at_stream = 0
        self.last_msg_time = None

        rospy.Subscriber(self.imu_topic, Imu, self.imu_cb, queue_size=10)
        rospy.on_shutdown(self.close)
        self.setup_keyboard()

    def setup_keyboard(self):
        if not self.keyboard_start:
            return
        try:
            self.tty = open("/dev/tty", "r")
            self.tty_settings = termios.tcgetattr(self.tty.fileno())
            tty.setcbreak(self.tty.fileno())
            rospy.loginfo("INS calib control: press 's' to start gyrocompass calibration.")
        except OSError as exc:
            self.keyboard_start = False
            rospy.logwarn("Keyboard start disabled: %s", exc)

    def imu_cb(self, _msg):
        self.msg_count += 1
        self.last_msg_time = rospy.Time.now()

    def open(self):
        if self.serial and self.serial.is_open:
            return
        self.serial = serial.Serial(self.port, self.baud, timeout=self.timeout)
        rospy.loginfo("INS calibration serial opened on %s @ %d baud.", self.port, self.baud)

    def close(self):
        if self.serial and self.serial.is_open:
            self.serial.close()
        if self.tty and self.tty_settings:
            termios.tcsetattr(self.tty.fileno(), termios.TCSADRAIN, self.tty_settings)
            self.tty.close()
            self.tty = None

    def valid_stream_hz(self):
        if self.stream_hz in ALLOWED_STREAM_HZ:
            return self.stream_hz
        nearest = min(ALLOWED_STREAM_HZ, key=lambda hz: abs(hz - self.stream_hz))
        rospy.logwarn(
            "Unsupported INS stream_output_hz=%s. Using nearest supported value: %s Hz.",
            self.stream_hz,
            nearest,
        )
        self.stream_hz = nearest
        return self.stream_hz

    def write_frame(self, frame, label):
        self.open()
        self.serial.write(frame)
        self.serial.flush()
        rospy.loginfo("Sent INS %s: %s", label, hex_bytes(frame))

    def write_frames(self, frames, label):
        self.open()
        data = b"".join(frames)
        self.serial.write(data)
        self.serial.flush()
        rospy.loginfo("Sent INS %s: %s", label, hex_bytes(data))

    def start_calibration(self):
        if self.started and not self.ready_reported:
            rospy.logwarn("IMU calibration is already running.")
            return
        if self.send_stop_before_stream:
            self.write_frame(output_control_frame(0x81, 0, 0), "stop stream")
            rospy.sleep(self.delay)
        if self.send_gyrocompass_start:
            self.write_frame(GYROCOMPASS_START_FRAME, "gyrocompass start")
            rospy.sleep(self.delay)

        self.started = True
        self.stream_sent = False
        self.ready_reported = False
        self.start_time = rospy.Time.now()
        self.msg_count_at_stream = self.msg_count
        rospy.loginfo(
            "IMU calibration started. Wait %.0f seconds (%.1f minutes). Stream %d Hz will be requested after calibration.",
            self.calibration_duration,
            self.calibration_duration / 60.0,
            self.valid_stream_hz(),
        )

    def send_stream_commands(self):
        hz = self.valid_stream_hz()
        if self.send_stop_before_stream:
            self.write_frame(output_control_frame(0x81, 0, 0), "stop stream")
            rospy.sleep(self.delay)
        self.write_frames(
            [
                output_control_frame(0x01, hz, 0),
                output_control_frame(0x00, hz, self.stream_data_mask),
            ],
            f"stream start/request {hz} Hz",
        )
        self.stream_sent = True
        self.msg_count_at_stream = self.msg_count

    def handle_keyboard(self):
        if not self.keyboard_start or not self.tty:
            return
        readable, _, _ = select.select([self.tty], [], [], 0)
        if readable and self.tty.read(1).lower() == "s":
            rospy.loginfo("Key 's' pressed: starting INS gyrocompass calibration.")
            self.start_calibration()

    def update(self):
        self.handle_keyboard()
        if self.auto_start and not self.started:
            self.start_calibration()
        if not self.started or self.ready_reported:
            return

        elapsed = (rospy.Time.now() - self.start_time).to_sec()
        remaining = max(0.0, self.calibration_duration - elapsed)
        if remaining > 0.0:
            rospy.loginfo_throttle(
                30.0,
                "IMU calibration in progress: %.0f seconds remaining.",
                remaining,
            )
            return

        if not self.stream_sent:
            rospy.loginfo("IMU calibration time elapsed. Sending stream command now.")
            self.send_stream_commands()
            return

        fresh_data = (
            self.last_msg_time is not None
            and self.msg_count > self.msg_count_at_stream
            and (rospy.Time.now() - self.last_msg_time).to_sec() <= self.ready_timeout
        )
        if fresh_data:
            rospy.loginfo("IMU calibration done. %s has fresh data and can be used.", self.imu_topic)
            self.ready_reported = True
        else:
            rospy.logwarn_throttle(
                5.0,
                "Stream command sent, but %s has no fresh data yet. Run imu_ins.launch if needed.",
                self.imu_topic,
            )

    def spin(self):
        rospy.loginfo(
            "INS calibration node ready: auto_start=%s, stream_output_hz=%d.",
            self.auto_start,
            self.valid_stream_hz(),
        )
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            try:
                self.update()
            except serial.SerialException as exc:
                rospy.logerr_throttle(2.0, "INS calibration serial error: %s", exc)
                self.close()
                rospy.sleep(1.0)
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("calib_imu")
    ImuCalibrationNode().spin()
