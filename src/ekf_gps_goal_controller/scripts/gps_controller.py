#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
import math
import tf
import utm

class GPSController:
    def __init__(self):
        rospy.init_node('gps_goal_controller')

        # ===== Nhập GPS goal =====
        self.goal_lat = float(input("Enter goal latitude: "))
        self.goal_lon = float(input("Enter goal longitude: "))

        # Convert goal GPS -> UTM
        self.goal_x, self.goal_y, self.goal_zone, self.goal_letter = utm.from_latlon(
            self.goal_lat, self.goal_lon
        )

        rospy.loginfo(f"Goal GPS: lat={self.goal_lat}, lon={self.goal_lon}")
        rospy.loginfo(f"Goal UTM: x={self.goal_x:.3f}, y={self.goal_y:.3f}")

        # Params
        self.k_v = rospy.get_param('~k_v', 0.5)
        self.k_w = rospy.get_param('~k_w', 1.5)
        self.max_linear = rospy.get_param('~max_linear', 0.5)
        self.max_angular = rospy.get_param('~max_angular', 0.2)
        self.goal_tolerance = rospy.get_param('~goal_tolerance', 1.0)

        # State
        self.current_x = None
        self.current_y = None
        self.current_yaw = None
        self.goal_reached = False

        # ROS
        rospy.Subscriber('/gps/filtered', NavSatFix, self.gps_callback)
        rospy.Subscriber('/imu/data', Imu, self.imu_callback) # lấy yaw từ IMU
        # rospy.Subscriber('/odometry/filtered_map', Odometry, self.odom_callback)  # chỉ lấy yaw
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        rospy.loginfo("GPS Goal Controller started...")

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))
    
    # ---------------------------- LẤY YAW TỪ IMU ----------------------------
    def imu_callback(self, msg):
        """Lấy yaw trực tiếp từ IMU"""
        orientation = msg.orientation
        q = [orientation.x, orientation.y, orientation.z, orientation.w]
        (_, _, yaw) = tf.transformations.euler_from_quaternion(q)

        self.current_yaw = yaw

    # def odom_callback(self, msg):
    #     """Lấy yaw từ odometry/EKF"""
    #     orientation = msg.pose.pose.orientation
    #     q = [orientation.x, orientation.y, orientation.z, orientation.w]
    #     (_, _, yaw) = tf.transformations.euler_from_quaternion(q)
    #     self.current_yaw = yaw

    # ---------------------------- LẤY LAT/LON ----------------------------
    def gps_callback(self, msg):
        """Lấy current GPS trực tiếp từ /gps/fix"""
        if self.goal_reached:
            return

        lat = msg.latitude
        lon = msg.longitude

        # Convert current GPS -> UTM
        try:
            current_x, current_y, zone, letter = utm.from_latlon(lat, lon)
        except Exception as e:
            rospy.logwarn(f"UTM convert failed: {e}")
            return

        self.current_x = current_x
        self.current_y = current_y

        # rospy.loginfo(f"Goal UTM_Curent: x={self.current_x:.3f}, y={self.current_y:.3f}")

        # Check yaw available
        if self.current_yaw is None:
            rospy.logwarn_throttle(2, "Waiting for yaw from /imu/data ...")
            return

        dx = self.goal_x - self.current_x
        dy = self.goal_y - self.current_y

        distance = math.sqrt(dx**2 + dy**2)
        angle_to_goal = math.atan2(dy, dx)
        angle_error = self.normalize_angle(angle_to_goal - self.current_yaw)

        cmd = Twist()

        if distance > self.goal_tolerance:
            # Nếu lệch góc lớn -> ưu tiên quay trước
            if abs(angle_error) > 0.3:
                cmd.linear.x = 0.0
            else:
                cmd.linear.x = min(self.max_linear, self.k_v * distance)

            cmd.angular.z = max(
                min(self.k_w * angle_error, self.max_angular),
                -self.max_angular
            )

            rospy.loginfo_throttle(
                1,
                f"Moving... Dist={distance:.2f} m | AngleErr={math.degrees(angle_error):.1f} deg"
            )
        else:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.goal_reached = True
            rospy.loginfo("Reached GPS goal!")

        self.cmd_pub.publish(cmd)

if __name__ == '__main__':
    try:
        GPSController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass