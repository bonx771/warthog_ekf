#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <robot_localization/FromLL.h>
#include <geographic_msgs/GeoPoint.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

bool sendGPSGoal(MoveBaseClient& ac, double lat, double lon)
{
    ros::NodeHandle nh;
    ros::ServiceClient fromLL_client = nh.serviceClient<robot_localization::FromLL>("/fromLL");

    robot_localization::FromLL srv;
    srv.request.ll_point.latitude  = lat;
    srv.request.ll_point.longitude = lon;
    srv.request.ll_point.altitude  = 0.0;

    if (!fromLL_client.call(srv))
    {
        ROS_ERROR("Failed to call /fromLL service");
        return false;
    }

    double x = srv.response.map_point.x;
    double y = srv.response.map_point.y;
    ROS_INFO("GPS (%.6f, %.6f) → map (%.2f, %.2f)", lat, lon, x, y);

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp    = ros::Time::now();
    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;
    goal.target_pose.pose.position.z = 0.0;
    goal.target_pose.pose.orientation.w = 1.0;

    ac.sendGoal(goal);
    ROS_INFO("Da gui goal, dang cho robot den...");

    bool finished = ac.waitForResult(ros::Duration(60.0));
    if (!finished)
    {
        ROS_ERROR("Timeout! Robot khong den dich sau 60s");
        ac.cancelGoal();
        return false;
    }

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("Den dich!");
        return true;
    }
    else
    {
        ROS_WARN("That bai, state: %s", ac.getState().toString().c_str());
        return false;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "gps_waypoint_sender_cpp");
    ros::NodeHandle nh;

    MoveBaseClient ac("move_base", true);
    ROS_INFO("Dang cho move_base action server...");
    ac.waitForServer();
    ROS_INFO("Connected!");

    std::vector<std::pair<double, double>> waypoints = {
        {21.0301, 105.8501},
        {21.0302, 105.8502},
    };

    for (const auto& wp : waypoints)
    {
        if (!ros::ok()) break;
        sendGPSGoal(ac, wp.first, wp.second);
        ros::Duration(1.0).sleep();
    }

    return 0;
}