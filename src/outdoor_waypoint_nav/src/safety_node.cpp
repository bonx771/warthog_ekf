#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

#include <cmath>
#include <limits>

ros::Publisher pubVel;

sensor_msgs::LaserScan latest_scan;
bool have_scan = false;
bool obstacle_avoidance_active = false;
double obstacle_avoidance_locked_turn_direction = 0.0;
ros::Time obstacle_avoidance_commit_until;

bool obstacle_avoidance_enabled = true;
double obstacle_avoidance_distance = 1.0;
double obstacle_avoidance_sector_half_angle_deg = 35.0;
double obstacle_avoidance_turn_speed = 0.6;
double obstacle_avoidance_forward_speed = 0.0;
double obstacle_avoidance_stop_distance = 0.8;
double obstacle_avoidance_min_cmd_linear_x = 0.05;
double obstacle_avoidance_clear_distance = 2.4;
double obstacle_avoidance_turn_commit_time = 1.0;

double normalizeAngle(double angle)
{
    while(angle > M_PI)
    {
        angle -= (2.0 * M_PI);
    }

    while(angle < -M_PI)
    {
        angle += (2.0 * M_PI);
    }

    return angle;
}

bool scanReadingIsUsable(const sensor_msgs::LaserScan& scan, float range)
{
    return std::isfinite(range) && range >= scan.range_min && range <= scan.range_max;
}

bool computeAvoidanceTurnDirection(const sensor_msgs::LaserScan& scan, double& turn_direction, double& front_min_range)
{
    const double sector_half_angle_rad = obstacle_avoidance_sector_half_angle_deg * M_PI / 180.0;
    front_min_range = std::numeric_limits<double>::infinity();
    double left_clearance = 0.0;
    double right_clearance = 0.0;
    int left_count = 0;
    int right_count = 0;

    for(std::size_t i = 0; i < scan.ranges.size(); ++i)
    {
        const float range = scan.ranges[i];
        if(!scanReadingIsUsable(scan, range))
        {
            continue;
        }

        const double angle = normalizeAngle(scan.angle_min + (static_cast<double>(i) * scan.angle_increment));
        if(std::fabs(angle) <= sector_half_angle_rad)
        {
            front_min_range = std::min(front_min_range, static_cast<double>(range));
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
    }

    if(!std::isfinite(front_min_range) || front_min_range > obstacle_avoidance_distance)
    {
        return false;
    }

    const double left_metric = (left_count > 0) ? (left_clearance / left_count) : 0.0;
    const double right_metric = (right_count > 0) ? (right_clearance / right_count) : 0.0;
    turn_direction = (left_metric >= right_metric) ? 1.0 : -1.0;
    return true;
}

void scanCB(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
    latest_scan = *scan_msg;
    have_scan = true;
}

void cmdVelCB(const geometry_msgs::Twist::ConstPtr& vel_msg)
{
    geometry_msgs::Twist output = *vel_msg;

    if(obstacle_avoidance_enabled &&
       have_scan &&
       output.linear.x >= obstacle_avoidance_min_cmd_linear_x)
    {
        double front_min_range = std::numeric_limits<double>::infinity();
        double suggested_turn_direction = 0.0;
        const bool obstacle_detected =
            computeAvoidanceTurnDirection(latest_scan, suggested_turn_direction, front_min_range);

        if(obstacle_detected && !obstacle_avoidance_active)
        {
            obstacle_avoidance_active = true;
            obstacle_avoidance_locked_turn_direction = suggested_turn_direction;
            obstacle_avoidance_commit_until = ros::Time::now() + ros::Duration(obstacle_avoidance_turn_commit_time);
            ROS_WARN(
                "Safety node: obstacle detected at %.2fm. Locking avoidance turn %s for %.2fs.",
                front_min_range,
                (obstacle_avoidance_locked_turn_direction > 0.0 ? "left" : "right"),
                obstacle_avoidance_turn_commit_time);
        }

        if(obstacle_avoidance_active)
        {
            if(!std::isfinite(front_min_range) || front_min_range > obstacle_avoidance_clear_distance)
            {
                obstacle_avoidance_active = false;
                obstacle_avoidance_locked_turn_direction = 0.0;
            }
            else
            {
                const bool commit_phase = ros::Time::now() < obstacle_avoidance_commit_until;
                if(commit_phase || front_min_range <= obstacle_avoidance_stop_distance)
                {
                    output.linear.x = 0.0;
                }
                else
                {
                    output.linear.x = std::min(output.linear.x, obstacle_avoidance_forward_speed);
                }
                output.angular.z =
                    obstacle_avoidance_locked_turn_direction * std::fabs(obstacle_avoidance_turn_speed);
                ROS_WARN_THROTTLE(
                    1.0,
                    "Safety node: obstacle at %.2fm, keeping committed turn %s (commit phase: %s).",
                    front_min_range,
                    (obstacle_avoidance_locked_turn_direction > 0.0 ? "left" : "right"),
                    (commit_phase ? "yes" : "no"));
            }
        }
    }
    else if(obstacle_avoidance_active)
    {
        obstacle_avoidance_active = false;
        obstacle_avoidance_locked_turn_direction = 0.0;
    }

    pubVel.publish(output);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "safety_node");
    ros::NodeHandle n;

    ros::param::param<bool>("/outdoor_waypoint_nav/obstacle_avoidance_enabled", obstacle_avoidance_enabled, true);
    ros::param::param<double>("/outdoor_waypoint_nav/obstacle_avoidance_distance", obstacle_avoidance_distance, 1.0);
    ros::param::param<double>("/outdoor_waypoint_nav/obstacle_avoidance_sector_half_angle_deg", obstacle_avoidance_sector_half_angle_deg, 35.0);
    ros::param::param<double>("/outdoor_waypoint_nav/obstacle_avoidance_turn_speed", obstacle_avoidance_turn_speed, 0.6);
    ros::param::param<double>("/outdoor_waypoint_nav/obstacle_avoidance_forward_speed", obstacle_avoidance_forward_speed, 0.0);
    ros::param::param<double>("/outdoor_waypoint_nav/obstacle_avoidance_stop_distance", obstacle_avoidance_stop_distance, 0.8);
    ros::param::param<double>("/outdoor_waypoint_nav/obstacle_avoidance_min_cmd_linear_x", obstacle_avoidance_min_cmd_linear_x, 0.05);
    ros::param::param<double>("/outdoor_waypoint_nav/obstacle_avoidance_clear_distance", obstacle_avoidance_clear_distance, 2.4);
    ros::param::param<double>("/outdoor_waypoint_nav/obstacle_avoidance_turn_commit_time", obstacle_avoidance_turn_commit_time, 1.0);

    ROS_INFO("Initiated safety_node");
    ros::Rate rate(50.0);

    pubVel = n.advertise<geometry_msgs::Twist>("/cmd_vel", 100);

    ros::Subscriber sub_cmd_vel = n.subscribe("/cmd_vel_intermediate", 100, cmdVelCB);
    ros::Subscriber sub_scan = n.subscribe("/front/scan", 20, scanCB);

    while(ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
