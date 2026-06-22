#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

ros::Publisher cmd_pub;

geometry_msgs::Twist latest_cmd;
sensor_msgs::LaserScan latest_scan;
ros::Time last_cmd_time(0);

bool have_cmd = false;
bool have_scan = false;
bool safety_enabled = true;

double stop_distance = 0.35;
double slowdown_distance = 0.80;
double sector_half_angle_deg = 20.0;
double command_timeout = 0.5;

std::string input_cmd_topic = "/cmd_vel_intermediate";
std::string output_cmd_topic = "/cmd_vel";
std::string scan_topic = "/front/scan";

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
    return std::isfinite(range) &&
           range >= scan.range_min &&
           range <= scan.range_max;
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

void publishStop()
{
    cmd_pub.publish(geometry_msgs::Twist());
}

double frontMinimumRange()
{
    if(!have_scan)
    {
        return std::numeric_limits<double>::infinity();
    }

    const double sector_half_angle_rad =
        std::fabs(sector_half_angle_deg) * M_PI / 180.0;
    double minimum_range = std::numeric_limits<double>::infinity();

    for(std::size_t i = 0; i < latest_scan.ranges.size(); ++i)
    {
        const float range = latest_scan.ranges[i];
        if(!scanReadingIsUsable(latest_scan, range))
        {
            continue;
        }

        const double angle = normalizeAngle(
            latest_scan.angle_min +
            static_cast<double>(i) * latest_scan.angle_increment);

        if(std::fabs(angle) <= sector_half_angle_rad)
        {
            minimum_range =
                std::min(minimum_range, static_cast<double>(range));
        }
    }

    return minimum_range;
}

double forwardSpeedScale(double obstacle_range)
{
    if(!std::isfinite(obstacle_range) ||
       obstacle_range >= slowdown_distance)
    {
        return 1.0;
    }

    if(obstacle_range <= stop_distance)
    {
        return 0.0;
    }

    const double available_distance = slowdown_distance - stop_distance;
    if(available_distance <= 1e-6)
    {
        return 0.0;
    }

    return std::max(
        0.0,
        std::min(
            1.0,
            (obstacle_range - stop_distance) / available_distance));
}

void controlTimerCB(const ros::TimerEvent&)
{
    const bool command_is_fresh =
        have_cmd &&
        (ros::Time::now() - last_cmd_time).toSec() <= command_timeout;

    if(!command_is_fresh)
    {
        publishStop();
        return;
    }

    if(!safety_enabled || !have_scan)
    {
        cmd_pub.publish(latest_cmd);
        return;
    }

    geometry_msgs::Twist output = latest_cmd;

    // move_base remains in full control of steering so the robot follows the
    // planned pink path. The safety node only limits forward motion near an
    // obstacle and never creates an independent turn/drive maneuver.
    if(output.linear.x > 0.0)
    {
        const double obstacle_range = frontMinimumRange();
        const double speed_scale = forwardSpeedScale(obstacle_range);
        output.linear.x *= speed_scale;

        if(speed_scale <= 0.0)
        {
            ROS_WARN_THROTTLE(
                1.0,
                "safety_node: obstacle at %.2fm; blocking forward motion "
                "while preserving move_base steering.",
                obstacle_range);
        }
        else if(speed_scale < 1.0)
        {
            ROS_INFO_THROTTLE(
                1.0,
                "safety_node: obstacle at %.2fm; forward speed scaled to %.0f%%.",
                obstacle_range,
                speed_scale * 100.0);
        }
    }

    cmd_pub.publish(output);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "safety_node");
    ros::NodeHandle nh;

    ros::param::param<bool>(
        "/outdoor_waypoint_nav/safety_enabled",
        safety_enabled,
        true);
    ros::param::param<double>(
        "/outdoor_waypoint_nav/safety_stop_distance",
        stop_distance,
        0.35);
    ros::param::param<double>(
        "/outdoor_waypoint_nav/safety_slowdown_distance",
        slowdown_distance,
        0.80);
    ros::param::param<double>(
        "/outdoor_waypoint_nav/safety_sector_half_angle_deg",
        sector_half_angle_deg,
        20.0);
    ros::param::param<double>(
        "/outdoor_waypoint_nav/safety_command_timeout",
        command_timeout,
        0.5);
    ros::param::param<std::string>(
        "/outdoor_waypoint_nav/safety_input_cmd_topic",
        input_cmd_topic,
        "/cmd_vel_intermediate");
    ros::param::param<std::string>(
        "/outdoor_waypoint_nav/safety_output_cmd_topic",
        output_cmd_topic,
        "/cmd_vel");
    ros::param::param<std::string>(
        "/outdoor_waypoint_nav/safety_scan_topic",
        scan_topic,
        "/front/scan");

    if(slowdown_distance <= stop_distance)
    {
        ROS_WARN(
            "safety_node: slowdown distance %.2fm must be larger than stop "
            "distance %.2fm. Using %.2fm.",
            slowdown_distance,
            stop_distance,
            stop_distance + 0.10);
        slowdown_distance = stop_distance + 0.10;
    }

    cmd_pub = nh.advertise<geometry_msgs::Twist>(output_cmd_topic, 10);

    ros::Subscriber cmd_sub = nh.subscribe(input_cmd_topic, 10, cmdCB);
    ros::Subscriber scan_sub = nh.subscribe(scan_topic, 10, scanCB);
    ros::Timer control_timer =
        nh.createTimer(ros::Duration(0.05), controlTimerCB);

    ROS_INFO(
        "safety_node: velocity gate active. input=%s output=%s scan=%s "
        "slowdown=%.2fm stop=%.2fm sector=+/-%.1fdeg",
        input_cmd_topic.c_str(),
        output_cmd_topic.c_str(),
        scan_topic.c_str(),
        slowdown_distance,
        stop_distance,
        sector_half_angle_deg);

    ros::spin();
    return 0;
}
