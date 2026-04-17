#include <ros/ros.h>
#include <ros/package.h>
#include <utility>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>
#include <robot_localization/navsat_conversions.h>
#include <tf/transform_listener.h>
#include <ros/duration.h>
#include <ros/time.h>
#include <math.h>



bool collect_request;
bool continue_collection = true;
double lati_point=0, longi_point=0, lati_last=0, longi_last=0;
double min_coord_change = 10 * pow(10,-6);
double marker_scale = 0.7;
std::string end_button_sym, collect_button_sym, gps_topic, marker_frame, marker_topic, utm_zone;
int end_button_num = 0, collect_button_num = 0;

geometry_msgs::PointStamped latLongToUTM(double lati_input, double longi_input)
{
	double utm_x = 0;
	double utm_y = 0;
	geometry_msgs::PointStamped utm_point;

	RobotLocalization::NavsatConversions::LLtoUTM(lati_input, longi_input, utm_y, utm_x, utm_zone);
	utm_point.header.frame_id = "utm";
	utm_point.header.stamp = ros::Time(0);
	utm_point.point.x = utm_x;
	utm_point.point.y = utm_y;
	utm_point.point.z = 0.0;
	return utm_point;
}

bool UTMToMarkerFrame(
	const geometry_msgs::PointStamped& utm_point,
	geometry_msgs::PointStamped& marker_point,
	tf::TransformListener& listener)
{
	try
	{
		listener.waitForTransform(marker_frame, "utm", ros::Time(0), ros::Duration(1.0));
		listener.transformPoint(marker_frame, utm_point, marker_point);
		return true;
	}
	catch (tf::TransformException& ex)
	{
		ROS_WARN("Saved waypoint, but could not transform marker into %s: %s", marker_frame.c_str(), ex.what());
		return false;
	}
}

void clearWaypointMarkers(ros::Publisher& marker_pub)
{
	visualization_msgs::Marker clear_marker;
	clear_marker.header.frame_id = marker_frame;
	clear_marker.header.stamp = ros::Time::now();
	clear_marker.action = visualization_msgs::Marker::DELETEALL;
	marker_pub.publish(clear_marker);
}

void publishWaypointMarker(
	ros::Publisher& marker_pub,
	const geometry_msgs::PointStamped& marker_point,
	int waypoint_index)
{
	visualization_msgs::Marker marker;
	marker.header.frame_id = marker_frame;
	marker.header.stamp = ros::Time::now();
	marker.ns = "collected_waypoints";
	marker.id = waypoint_index;
	marker.type = visualization_msgs::Marker::SPHERE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.orientation.w = 1.0;
	marker.pose.position.x = marker_point.point.x;
	marker.pose.position.y = marker_point.point.y;
	marker.pose.position.z = marker_point.point.z + 0.2;
	marker.scale.x = marker_scale;
	marker.scale.y = marker_scale;
	marker.scale.z = marker_scale;
	marker.color.a = 1.0;
	marker.color.r = 1.0;
	marker.color.g = 0.0;
	marker.color.b = 0.0;
	marker.lifetime = ros::Duration(0.0);
	marker_pub.publish(marker);
}

void joy_CB(const sensor_msgs::Joy joy_msg)
{
	if(joy_msg.buttons[collect_button_num]==1)
	{
		collect_request = true;
	}
	else
	{
		collect_request = false;
	}

	if(joy_msg.buttons[end_button_num]==1)
	{
		continue_collection = false;
	}
}

void filtered_gps_CB(const sensor_msgs::NavSatFix gps_msg)
{
		lati_point = gps_msg.latitude;
		longi_point = gps_msg.longitude;
}

int main(int argc, char** argv)
{
	//Initialize variables
		int numWaypoints = 0;
		std::string path_local;

    // Initialize node and time
		ros::init(argc, argv, "collect_gps_waypoints"); //initiate node called collect_gps_waypoints
		ros::NodeHandle n;
		ros::Time::init();
		ros::Time time_last;
		ros::Time time_current;
		ros::Duration duration_min(1);

	// Get button numbers to collect waypoints and end collection
		ros::param::get("/outdoor_waypoint_nav/collect_button_num", collect_button_num);
		ros::param::get("/outdoor_waypoint_nav/end_button_num", end_button_num);
		ros::param::param<std::string>("/outdoor_waypoint_nav/gps_topic", gps_topic, "/outdoor_waypoint_nav/gps/filtered");
		ros::param::param<std::string>("/outdoor_waypoint_nav/waypoint_marker_frame", marker_frame, "map");
		ros::param::param<std::string>("/outdoor_waypoint_nav/waypoint_marker_topic", marker_topic, "/outdoor_waypoint_nav/collected_waypoints");
		ros::param::param<double>("/outdoor_waypoint_nav/waypoint_marker_scale", marker_scale, 0.7);

    //Initiate subscribers
		ros::Subscriber sub_joy = n.subscribe("/joy_teleop/joy", 100, joy_CB);
		ros::Subscriber sub_gps = n.subscribe(gps_topic, 100, filtered_gps_CB);
		ros::Publisher pubWaypointMarkers = n.advertise<visualization_msgs::Marker>(marker_topic, 100, true);
		tf::TransformListener listener;
		ROS_INFO("Initiated collect_gps_waypoints node");
		ROS_INFO("Collecting waypoints from GPS topic: %s", gps_topic.c_str());
		ROS_INFO("Publishing collected waypoint markers to: %s", marker_topic.c_str());

	// Initiate publisher to send end of node message
		ros::Publisher pubCollectionNodeEnded = n.advertise<std_msgs::Bool>("/outdoor_waypoint_nav/collection_status",100);

    //Read file path and create/open file
    	ros::param::get("/outdoor_waypoint_nav/coordinates_file", path_local);
		std::string path_abs =  ros::package::getPath("outdoor_waypoint_nav") + path_local;	
		std::ofstream coordFile (path_abs.c_str());
		ROS_INFO("Saving coordinates to: %s", path_abs.c_str());
		
	// Give instructions:
		ros::param::get("/outdoor_waypoint_nav/collect_button_sym", collect_button_sym);
		ros::param::get("/outdoor_waypoint_nav/end_button_sym", end_button_sym);
		std::cout << std::endl;
		std::cout << "Press " << collect_button_sym.c_str() << " button to collect and store waypoint." << std::endl;
		std::cout << "Press " << end_button_sym.c_str() << " button to end waypoint collection." << std::endl;
		std::cout << std::endl;
		clearWaypointMarkers(pubWaypointMarkers);
		ros::Duration(0.1).sleep();

	if(coordFile.is_open())
	{
		while(continue_collection)
		{
			ros::spinOnce();
			time_current = ros::Time::now();
			if((collect_request == true) && (time_current - time_last > duration_min))
			{	
				// Check that there was sufficient change in position between points
				// This makes the move_base navigation smoother and stops points from being collected twice
				double difference_lat = abs((lati_point - lati_last)*pow(10,6))*pow(10,-6);
				double difference_long = abs((longi_point - longi_last)*pow(10,6))*pow(10,-6);

				if( (difference_lat > min_coord_change) || (difference_long > min_coord_change))
				{
					numWaypoints++;
					coordFile << std::fixed << std::setprecision(8) << lati_point << " " << longi_point << std::endl;
					lati_last = lati_point;
					longi_last = longi_point;

					geometry_msgs::PointStamped marker_point;
					geometry_msgs::PointStamped utm_point = latLongToUTM(lati_point, longi_point);
					if(UTMToMarkerFrame(utm_point, marker_point, listener))
					{
						publishWaypointMarker(pubWaypointMarkers, marker_point, numWaypoints);
					}

						ROS_INFO("Collected waypoint %d (lat, lon): %.8f, %.8f", numWaypoints, lati_point, longi_point);
						ROS_INFO("Press %s button to collect next waypoint %d.", collect_button_sym.c_str(), numWaypoints + 1);
						ROS_INFO("Press %s button to end waypoint collection.", end_button_sym.c_str());
					std::cout << std::endl;
				}

				else
				{//do not write waypoint
					ROS_WARN("Waypoint not saved, you have not moved enough");
					ROS_WARN("New Latitude: %f   Last Latitude: %f \n", lati_point, lati_last );
					ROS_WARN("New Longitude: %f   Last Longitude: %f \n", longi_point, longi_last );
				}
				time_last = time_current;
			}
			else{}
			ros::spinOnce();
		}
	
		coordFile.close();
		ROS_INFO("End request registered.");
	}
	else
	{
		ROS_ERROR("Unable to open file.");
		ROS_INFO("Exiting..");
	}

	ROS_INFO("Closed waypoint file, you have collected %d waypoints.", numWaypoints);
	ROS_INFO("Waypoint collection complete.");
	ROS_INFO("Ending node...");

	// Notify joy_launch_control that calibration is complete
    std_msgs::Bool node_ended;
    node_ended.data = true;
    pubCollectionNodeEnded.publish(node_ended);

	ros::shutdown();
	return 0;
}
