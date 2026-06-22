#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>
#include <tf/transform_listener.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

enum Mode
{
    MODE_PASS_THROUGH = 0,
    MODE_TURNING,
    MODE_DRIVING
};

ros::Publisher cmd_pub;
ros::Publisher replan_pub;
tf::TransformListener* tf_listener_ptr = NULL;

geometry_msgs::Twist latest_cmd;
sensor_msgs::LaserScan latest_scan;
nav_msgs::Path latest_plan;
ros::Time last_cmd_time(0);
ros::Time last_maneuver_finish_time(0);

bool have_cmd = false;
bool have_scan = false;
bool have_plan = false;

Mode mode = MODE_PASS_THROUGH;
double turn_direction = 1.0;
double start_yaw = 0.0;
double start_x = 0.0;
double start_y = 0.0;
ros::Time phase_start_time(0);
bool have_start_pose = false;

bool safety_enabled = true;
double obstacle_distance = 1.6;
double sector_half_angle_deg = 45.0;
double turn_angle_deg = 30.0;
double forward_distance = 2.0;
double turn_speed = 0.45;
double forward_speed = 0.45;
double command_timeout = 0.5;
double maneuver_cooldown = 1.0;
double path_lookahead_distance = 1.0;
double planner_active_linear_threshold = 0.02;
double planner_active_angular_threshold = 0.05;
std::string input_cmd_topic = "/cmd_vel_intermediate";
std::string output_cmd_topic = "/cmd_vel";
std::string scan_topic = "/front/scan";
std::string plan_topic = "/move_base/NavfnROS/plan";
std::string replan_topic = "/outdoor_waypoint_nav/replan_requested";
std::string odom_frame = "odom";
std::string base_frame = "base_link";

double normalizeAngle(double angle)
{
    while(angle > M_PI)
    {
        angle -= 2.0 * M_PI;
    }
    while(angle < -M_PI)
    {
        angle += 2.0 * M_PI;
    }
    return angle;
}

bool scanReadingIsUsable(const sensor_msgs::LaserScan& scan, float range)
{
    return std::isfinite(range) && range >= scan.range_min && range <= scan.range_max;
}

void cmdCB(const geometry_msgs::Twist::ConstPtr& cmd_msg)
{
    latest_cmd = *cmd_msg;
    have_cmd = true;
    last_cmd_time = ros::Time::now();
}

void scanCB(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
    latest_scan = *scan_msg;
    have_scan = true;
}

void planCB(const nav_msgs::Path::ConstPtr& plan_msg)
{
    latest_plan = *plan_msg;
    have_plan = true;
}

bool tryGetRobotPose(double& x, double& y, double& yaw)
{
    if(tf_listener_ptr == NULL)
    {
        return false;
    }

    tf::StampedTransform transform;
    try
    {
        tf_listener_ptr->lookupTransform(odom_frame, base_frame, ros::Time(0), transform);
        x = transform.getOrigin().x();
        y = transform.getOrigin().y();
        yaw = tf::getYaw(transform.getRotation());
        return true;
    }
    catch(tf::TransformException& ex)
    {
        ROS_WARN_THROTTLE(2.0, "safety_node: cannot read robot pose: %s", ex.what());
        return false;
    }
}

bool frontObstacleDetected(double& front_min_range)
{
    if(!have_scan)
    {
        return false;
    }

    const double sector_half_angle_rad = sector_half_angle_deg * M_PI / 180.0;
    front_min_range = std::numeric_limits<double>::infinity();

    for(std::size_t i = 0; i < latest_scan.ranges.size(); ++i)
    {
        const float range = latest_scan.ranges[i];
        if(!scanReadingIsUsable(latest_scan, range))
        {
            continue;
        }

        const double angle =
            normalizeAngle(latest_scan.angle_min + static_cast<double>(i) * latest_scan.angle_increment);
        if(std::fabs(angle) <= sector_half_angle_rad)
        {
            front_min_range = std::min(front_min_range, static_cast<double>(range));
        }
    }

    return std::isfinite(front_min_range) && front_min_range <= obstacle_distance;
}

bool chooseTurnDirectionFromPlan(double& direction)
{
    if(!have_plan || latest_plan.poses.empty() || tf_listener_ptr == NULL)
    {
        return false;
    }

    bool found = false;
    double best_distance = std::numeric_limits<double>::infinity();
    double best_y = 0.0;

    for(std::size_t i = 0; i < latest_plan.poses.size(); ++i)
    {
        geometry_msgs::PoseStamped plan_pose = latest_plan.poses[i];
        if(plan_pose.header.frame_id.empty())
        {
            plan_pose.header.frame_id = latest_plan.header.frame_id;
        }
        plan_pose.header.stamp = ros::Time(0);

        geometry_msgs::PoseStamped base_pose;
        try
        {
            tf_listener_ptr->transformPose(base_frame, plan_pose, base_pose);
        }
        catch(tf::TransformException&)
        {
            continue;
        }

        const double x = base_pose.pose.position.x;
        const double y = base_pose.pose.position.y;
        const double distance = std::sqrt((x * x) + (y * y));

        if(x <= 0.2 || distance < path_lookahead_distance)
        {
            continue;
        }

        if(distance < best_distance)
        {
            best_distance = distance;
            best_y = y;
            found = true;
        }
    }

    if(!found)
    {
        return false;
    }

    direction = best_y >= 0.0 ? 1.0 : -1.0;
    return true;
}

double chooseTurnDirectionFromScan()
{
    double left_clearance = 0.0;
    double right_clearance = 0.0;
    int left_count = 0;
    int right_count = 0;

    for(std::size_t i = 0; i < latest_scan.ranges.size(); ++i)
    {
        const float range = latest_scan.ranges[i];
        if(!scanReadingIsUsable(latest_scan, range))
        {
            continue;
        }

        const double angle =
            normalizeAngle(latest_scan.angle_min + static_cast<double>(i) * latest_scan.angle_increment);
        if(std::fabs(angle) > M_PI / 2.0)
        {
            continue;
        }

        if(angle >= 0.0)
        {
            left_clearance += range;
            left_count++;
        }
        else
        {
            right_clearance += range;
            right_count++;
        }
    }

    const double left_metric = left_count > 0 ? left_clearance / left_count : 0.0;
    const double right_metric = right_count > 0 ? right_clearance / right_count : 0.0;
    return left_metric >= right_metric ? 1.0 : -1.0;
}

double chooseTurnDirection()
{
    double direction = 1.0;
    if(chooseTurnDirectionFromPlan(direction))
    {
        return direction;
    }
    return chooseTurnDirectionFromScan();
}

bool plannerCommandIsActive()
{
    if(!have_cmd || (ros::Time::now() - last_cmd_time).toSec() > command_timeout)
    {
        return false;
    }

    return std::fabs(latest_cmd.linear.x) >= planner_active_linear_threshold ||
           std::fabs(latest_cmd.angular.z) >= planner_active_angular_threshold;
}

void publishStop()
{
    geometry_msgs::Twist stop_cmd;
    cmd_pub.publish(stop_cmd);
}

void publishReplanRequest()
{
    std_msgs::Bool request;
    request.data = true;
    replan_pub.publish(request);
}

void startManeuver(double front_min_range)
{
    turn_direction = chooseTurnDirection();
    double unused_yaw = 0.0;
    have_start_pose = tryGetRobotPose(start_x, start_y, unused_yaw);
    start_yaw = unused_yaw;
    phase_start_time = ros::Time::now();
    mode = MODE_TURNING;

    ROS_WARN(
        "safety_node: obstacle at %.2fm. Turning %s %.1fdeg toward global plan, then driving %.2fm.",
        front_min_range,
        turn_direction > 0.0 ? "left" : "right",
        turn_angle_deg,
        forward_distance);
}

bool turnComplete()
{
    const double target_angle = std::fabs(turn_angle_deg) * M_PI / 180.0;
    const double elapsed = (ros::Time::now() - phase_start_time).toSec();
    const double expected_time = target_angle / std::max(0.05, std::fabs(turn_speed));

    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    if(have_start_pose && tryGetRobotPose(x, y, yaw))
    {
        const double signed_turn = turn_direction * normalizeAngle(yaw - start_yaw);
        if(signed_turn >= target_angle)
        {
            return true;
        }
    }

    return elapsed >= expected_time * 1.5;
}

bool driveComplete()
{
    const double elapsed = (ros::Time::now() - phase_start_time).toSec();
    const double expected_time = forward_distance / std::max(0.05, std::fabs(forward_speed));

    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    if(have_start_pose && tryGetRobotPose(x, y, yaw))
    {
        const double dx = x - start_x;
        const double dy = y - start_y;
        if(std::sqrt((dx * dx) + (dy * dy)) >= forward_distance)
        {
            return true;
        }
    }

    return elapsed >= expected_time * 1.5;
}

void finishManeuver()
{
    mode = MODE_PASS_THROUGH;
    last_maneuver_finish_time = ros::Time::now();
    publishStop();
    publishReplanRequest();
    ROS_WARN("safety_node: avoidance maneuver finished. Requested move_base replan from current pose.");
}

void controlTimerCB(const ros::TimerEvent&)
{
    if(!safety_enabled)
    {
        if(have_cmd)
        {
            cmd_pub.publish(latest_cmd);
        }
        return;
    }

    if(mode == MODE_PASS_THROUGH)
    {
        double front_min_range = std::numeric_limits<double>::infinity();
        const bool obstacle_detected = frontObstacleDetected(front_min_range);
        const bool cooldown_done =
            last_maneuver_finish_time.toSec() <= 0.0 ||
            (ros::Time::now() - last_maneuver_finish_time).toSec() >= maneuver_cooldown;

        if(obstacle_detected && plannerCommandIsActive() && cooldown_done)
        {
            startManeuver(front_min_range);
        }
        else
        {
            if(have_cmd && (ros::Time::now() - last_cmd_time).toSec() <= command_timeout)
            {
                cmd_pub.publish(latest_cmd);
            }
            else
            {
                publishStop();
            }
            return;
        }
    }

    geometry_msgs::Twist output;
    if(mode == MODE_TURNING)
    {
        output.angular.z = turn_direction * std::fabs(turn_speed);
        cmd_pub.publish(output);

        if(turnComplete())
        {
            have_start_pose = tryGetRobotPose(start_x, start_y, start_yaw);
            phase_start_time = ros::Time::now();
            mode = MODE_DRIVING;
            ROS_WARN("safety_node: turn complete. Driving straight %.2fm.", forward_distance);
        }
    }
    else if(mode == MODE_DRIVING)
    {
        output.linear.x = std::fabs(forward_speed);
        cmd_pub.publish(output);

        if(driveComplete())
        {
            finishManeuver();
        }
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "safety_node");
    ros::NodeHandle nh;

    ros::param::param<bool>("/outdoor_waypoint_nav/safety_enabled", safety_enabled, true);
    ros::param::param<double>("/outdoor_waypoint_nav/safety_obstacle_distance", obstacle_distance, 1.6);
    ros::param::param<double>("/outdoor_waypoint_nav/safety_sector_half_angle_deg", sector_half_angle_deg, 45.0);
    ros::param::param<double>("/outdoor_waypoint_nav/safety_turn_angle_deg", turn_angle_deg, 30.0);
    ros::param::param<double>("/outdoor_waypoint_nav/safety_forward_distance", forward_distance, 2.0);
    ros::param::param<double>("/outdoor_waypoint_nav/safety_turn_speed", turn_speed, 0.45);
    ros::param::param<double>("/outdoor_waypoint_nav/safety_forward_speed", forward_speed, 0.45);
    ros::param::param<double>("/outdoor_waypoint_nav/safety_command_timeout", command_timeout, 0.5);
    ros::param::param<double>("/outdoor_waypoint_nav/safety_maneuver_cooldown", maneuver_cooldown, 1.0);
    ros::param::param<double>("/outdoor_waypoint_nav/safety_path_lookahead_distance", path_lookahead_distance, 1.0);
    ros::param::param<std::string>("/outdoor_waypoint_nav/safety_input_cmd_topic", input_cmd_topic, "/cmd_vel_intermediate");
    ros::param::param<std::string>("/outdoor_waypoint_nav/safety_output_cmd_topic", output_cmd_topic, "/cmd_vel");
    ros::param::param<std::string>("/outdoor_waypoint_nav/safety_scan_topic", scan_topic, "/front/scan");
    ros::param::param<std::string>("/outdoor_waypoint_nav/safety_plan_topic", plan_topic, "/move_base/NavfnROS/plan");
    ros::param::param<std::string>("/outdoor_waypoint_nav/safety_replan_topic", replan_topic, "/outdoor_waypoint_nav/replan_requested");
    ros::param::param<std::string>("/outdoor_waypoint_nav/safety_odom_frame", odom_frame, "odom");
    ros::param::param<std::string>("/outdoor_waypoint_nav/safety_base_frame", base_frame, "base_link");

    tf::TransformListener tf_listener;
    tf_listener_ptr = &tf_listener;

    cmd_pub = nh.advertise<geometry_msgs::Twist>(output_cmd_topic, 10);
    replan_pub = nh.advertise<std_msgs::Bool>(replan_topic, 10);

    ros::Subscriber cmd_sub = nh.subscribe(input_cmd_topic, 10, cmdCB);
    ros::Subscriber scan_sub = nh.subscribe(scan_topic, 10, scanCB);
    ros::Subscriber plan_sub = nh.subscribe(plan_topic, 5, planCB);
    ros::Timer control_timer = nh.createTimer(ros::Duration(0.05), controlTimerCB);

    ROS_INFO(
        "safety_node: input=%s output=%s scan=%s plan=%s",
        input_cmd_topic.c_str(),
        output_cmd_topic.c_str(),
        scan_topic.c_str(),
        plan_topic.c_str());

    ros::spin();
    return 0;
}
