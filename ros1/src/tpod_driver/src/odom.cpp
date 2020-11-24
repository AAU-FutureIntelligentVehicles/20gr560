#include "ros/ros.h"
#include "std_msgs/String.h"
#include <tf/transform_broadcaster.h>
#include "serial/serial.h"
#include <sstream>
#include <iostream>   // std::cout
#include <cstring>     // std::string, std::stof
#include <math.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int64.h>

double left, right = 0;
double left_old, right_old = 0;
double delta_left, delta_right = 0;
double yaw = 0;
double x, y, delta_x, delta_y = 0;





void ticks_left_sub_callback(const std_msgs::Int64::ConstPtr& new_msg)
{
	left = new_msg->data;
}

void ticks_right_sub_callback(const std_msgs::Int64::ConstPtr& new_msg)
{
  right = new_msg->data;
	static tf::TransformBroadcaster br;
 
	// === Do the math ===	
	left = left * 0.00339271; // convert from tick to meter
  right = right * 0.00353982; 
    
  delta_left = (left - left_old); // Difference in movement since last iteration
	delta_right = (right - right_old); //The odd numbers comes from calibration

  left_old = left; // save value from this iteration
  right_old = right;

  yaw = (right - left)/0.98; // Calculate yaw (0.98 = wheelbase)

	delta_x = ((delta_left + delta_right)/2) * cos(yaw); // calculate difference in x movement
	delta_y = ((delta_left + delta_right)/2) * sin(yaw); // calculate difference in y movement
	
	x = x + delta_x; // calculate accumulated x movement
	y = y + delta_y; // calculate accumulated y movement

	std::cout<<left<<" "<<right<<" "<<yaw<<" "<<x<<" "<<y<<std::endl;

	

  // === Broadcast transformation ===
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(x, y, 0.22) ); //Sæt X og Y her. (Ingen bevægelse i Z-retning.)
  tf::Quaternion q;
  q.setRPY(0, 0, yaw); //Sæt Roll, Pitch, Yaw her. (Kun Yaw er aktuelt.) (Radian)
  
	transform.setRotation(q);

	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "rear_axle"));
}


int main(int argc, char **argv) 
{
  ros::init(argc, argv, "odom");

  ros::NodeHandle ros_nh;
  
  ros::Subscriber ticks_left_sub = ros_nh.subscribe("ticks_left", 1000, ticks_left_sub_callback);
  ros::Subscriber ticks_right_sub = ros_nh.subscribe("ticks_right", 1000, ticks_right_sub_callback);

	ros::spin();

  return 0;
}

