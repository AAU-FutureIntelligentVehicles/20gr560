#include <ros/ros.h>
#include <ros/console.h>
#include <sstream>
#include <iostream>
#include <std_msgs/Int64.h>
#include <std_msgs/Float64.h>
#include <math.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

double left = 0;
double right = 0;
double p_left, p_right = 0;
double delta_left, delta_right = 0;
double heading, currentHeading, previousHeading = 0;
double x, y, delta_x, delta_y = 0;
double v_left, v_right, v_x, v_heading = 0;

double rearTread = 0.98; //m
double wheelDiameter = 0.453; //m
double wheelCircumference = 1.42314147208; //m
double ticksPerRotation_Left = 317;
double ticksPerRotation_Right = 395;
double tickLength_Left = wheelCircumference/ticksPerRotation_Left;
double tickLength_Right = wheelCircumference/ticksPerRotation_Right;
double axleLength = 0;
double current_Time;
ros::Time previousTime;
double previous_Time;
ros::Time currentTime;
double timeChange;

nav_msgs::Odometry odom;

void LeftTicks_Callback(const std_msgs::Int64::ConstPtr& tick_msg)
{
	left = tick_msg->data;
}

void RightTicks_Callback(const std_msgs::Int64::ConstPtr& tick_msg)
{
  right = tick_msg->data;
}


void calculateOdometry(){
  currentTime = ros::Time::now();
  current_Time = ros::Time::now().toSec();

  //Convert ticks to length
  left = left * tickLength_Left;
  right = right * tickLength_Right;

  //Find change in position since last iteration
  delta_left = left - p_left;
  delta_right = right - p_right;

  //Find heading
  heading = (delta_right - delta_left) / rearTread;
	//Update Current Heading
	currentHeading = currentHeading + heading;

  //Find change in XY position since last iteration
  //Since we are looking at the center of the axle, we take the average of both left and right ticks
  //axleLength = ((delta_left+delta_right)/2);
	double delta_s = (delta_right + delta_left)/2;

	double delta_d = delta_s;
	delta_x = delta_d*cos(currentHeading+(heading/2));
	delta_y = delta_d*sin(currentHeading+(heading/2));
  //delta_x = axleLength * cos(currentHeading);
  //delta_y = axleLength * sin(currentHeading);
	//std::cout<<delta_x<<std::endl;

  //Update current position
  x = x + delta_x;
	y = y + delta_y;

	timeChange = current_Time - previous_Time;

  v_left = delta_left / timeChange;
  v_right = delta_right / timeChange;
  v_x = (v_left + v_right)/2.0; // meter/s

  v_heading = (currentHeading - previousHeading) * (current_Time - previous_Time);

  previousHeading = currentHeading;
  previousTime = currentTime;
  previous_Time = current_Time;
	//Update previous length
  p_left = left;
  p_right = right;
}

void odomPublisher(){
  //Create broadcaster to send odometry msg
  tf::TransformBroadcaster odom_broadcaster;

  geometry_msgs::Quaternion quaternion = tf::createQuaternionMsgFromYaw(currentHeading);

  geometry_msgs::TransformStamped transform;
  transform.header.stamp = currentTime;
  transform.header.frame_id = "odom";
  transform.child_frame_id = "rear_axle_center";
  transform.transform.translation.x = x;
  transform.transform.translation.y = y;
  transform.transform.translation.z = 0.0;
  transform.transform.rotation = quaternion;

  odom_broadcaster.sendTransform(transform);

  odom.header.stamp = currentTime;
  odom.header.frame_id = "odom";
  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = quaternion;
  odom.child_frame_id = "base_link";
  odom.twist.twist.linear.x = v_x;
  odom.twist.twist.linear.y = 0;
  odom.twist.twist.angular.z = v_heading;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "golf_odom");
  ros::NodeHandle nh;

	ros::Time previousTime = ros::Time::now();
	double previous_Time = ros::Time::now().toSec();

  ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("golf_odom", 50);
	ros::Publisher vel_pub = nh.advertise<std_msgs::Float64>("vel_odom",1);
	ros::Publisher dist = nh.advertise<std_msgs::Float64>("dist", 1);
	std_msgs::Float64 vel_msg;
	std_msgs::Float64 dist_msg;

	ros::Subscriber LeftTicks_sub = nh.subscribe("ticks_left", 1000, LeftTicks_Callback);
	ros::Subscriber RightTicks_sub = nh.subscribe("ticks_right", 1000, RightTicks_Callback);

  ros::Rate rate(5);
  while(nh.ok()){
    ros::spinOnce();

	  calculateOdometry();

    odomPublisher();
    odom_pub.publish(odom);
		vel_msg.data = v_x;
		dist_msg.data = left;
		dist.publish(dist_msg);
		vel_pub.publish(vel_msg);
    rate.sleep();
  }

  return 0;
}
