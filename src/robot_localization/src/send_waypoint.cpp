#include <ros/ros.h>
#include <vector>
#include <utility>
#include <cmath>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <robot_localization/FromLL.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class GpsNavigation {
public:
    GpsNavigation() : ac_("move_base", true) {
        ROS_INFO("Waiting for move_base action server...");
        ac_.waitForServer();

        fromLL_client_ = nh_.serviceClient<robot_localization::FromLL>("/fromLL");
        ROS_INFO("Waiting for /fromLL service...");
        fromLL_client_.waitForExistence();
    }

    // Hàm tính khoảng cách Euclid
    double getDistance(double x1, double y1, double x2, double y2) {
        return std::hypot(x2 - x1, y2 - y1);
    }

    // Lấy vị trí hiện tại của robot từ TF
    bool getCurrentPose(double& x, double& y, double& yaw) {
        tf::StampedTransform transform;
        try {
            // Đợi tối đa 1s để có transform map -> base_link
            listener_.waitForTransform("map", "base_link", ros::Time(0), ros::Duration(1.0));
            listener_.lookupTransform("map", "base_link", ros::Time(0), transform);
            
            x = transform.getOrigin().x();
            y = transform.getOrigin().y();
            yaw = tf::getYaw(transform.getRotation());
            return true;
        }
        catch (tf::TransformException &ex) {
            ROS_WARN("TF Error: %s", ex.what());
            return false;
        }
    }

    bool sendGoal(double lat, double lon, double next_lat, double next_lon, bool is_last) {
        // 1. Chuyển đổi GPS của đích hiện tại sang tọa độ XY (Map)
        robot_localization::FromLL srv;
        srv.request.ll_point.latitude = lat;
        srv.request.ll_point.longitude = lon;
        srv.request.ll_point.altitude = 0.0;

        if (!fromLL_client_.call(srv)) {
            ROS_ERROR("Cannot call /fromLL. Is navsat_transform_node running?");
            return false;
        }

        double goal_x = srv.response.map_point.x;
        double goal_y = srv.response.map_point.y;

        // 2. Lấy vị trí hiện tại để tính Yaw
        double curr_x, curr_y, curr_yaw;
        if (!getCurrentPose(curr_x, curr_y, curr_yaw)) {
            ROS_ERROR("Could not get current pose to calculate yaw.");
            return false;
        }

        // 3. Logic tính Target Yaw mới
        double target_yaw = 0.0;
        double dist_to_goal = getDistance(curr_x, curr_y, goal_x, goal_y);

        // LUÔN LUÔN hướng về phía đích hiện tại để robot đi thẳng bám đường
        target_yaw = atan2(goal_y - curr_y, goal_x - curr_x);

        // NẾU robot đã ở khá gần đích (ví dụ < 1.5m) và không phải điểm cuối
        // THÌ bắt đầu tính hướng xoay dần sang điểm tiếp theo để cua mượt
        if (!is_last && dist_to_goal < 1.5) {
            robot_localization::FromLL srv_next;
            srv_next.request.ll_point.latitude = next_lat;
            srv_next.request.ll_point.longitude = next_lon;
            if (fromLL_client_.call(srv_next)) {
                double next_x = srv_next.response.map_point.x;
                double next_y = srv_next.response.map_point.y;
                target_yaw = atan2(next_y - goal_y, next_x - goal_x);
                ROS_INFO("🔄 Approaching waypoint, look-ahead to next point...");
            }
        }

        // 4. Gửi Goal
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x = goal_x;
        goal.target_pose.pose.position.y = goal_y;
        
        // Tạo Quaternion từ target_yaw đã tính
        goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(target_yaw);

        ROS_INFO("🚀 Goal: (%.6f, %.6f) | Map: X=%.2f, Y=%.2f | Dist: %.2f m", 
                 lat, lon, goal_x, goal_y, dist_to_goal);

        ac_.sendGoal(goal);
        ac_.waitForResult();

        if (ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("✅ Waypoint Reached!");
            return true;
        } else {
            ROS_ERROR("❌ Failed to reach waypoint. State: %s", ac_.getState().toString().c_str());
            return false;
        }
    }

private:
    ros::NodeHandle nh_;
    tf::TransformListener listener_;
    MoveBaseClient ac_;
    ros::ServiceClient fromLL_client_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "gps_nav_node");
    GpsNavigation nav;

    // Danh sách tọa độ GPS thực tế của bạn
    std::vector<std::pair<double, double>> waypoints = {
        {21.035510, 105.868710}, 
        {21.035550, 105.868750}  
    };

    for (size_t i = 0; i < waypoints.size(); ++i) {
        if (!ros::ok()) break;

        bool is_last = (i == waypoints.size() - 1);
        double n_lat = is_last ? 0.0 : waypoints[i+1].first;
        double n_lon = is_last ? 0.0 : waypoints[i+1].second;

        ROS_INFO("--- Executing Waypoint %zu/%zu ---", i+1, waypoints.size());
        if (!nav.sendGoal(waypoints[i].first, waypoints[i].second, n_lat, n_lon, is_last)) {
            ROS_ERROR("Mission Aborted at waypoint %zu", i+1);
            break;
        }
        
        // Nghỉ một chút giữa các waypoint để hệ thống ổn định
        ros::Duration(1.5).sleep();
    }

    ROS_INFO("🏁 Mission Finished!");
    return 0;
}