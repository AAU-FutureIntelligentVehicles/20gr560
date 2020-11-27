#include <ros/ros.h>
#include <ros/console.h>
#include <sstream>
#include <iostream>
#include <std_msgs/Int64.h>
#include <std_msgs/Float64.h>
#include <math.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <cmath>

double linear_vel;
double wheel_base = 1.64;
double steering_angle, steering_angle_map;
double path_lin_vel, path_ang_vel;

std_msgs::Float64 lin_vel_pub;
std_msgs::Float64 ang_vel_pub;

void SetLinearVelocity(double desired_lin_vel){
  linear_vel = 0.6252*desired_lin_vel + 0.4173;
}

void SetSteeringWheelPosition(double desired_ang_vel){
  steering_angle = atan2((wheel_base*desired_ang_vel), linear_vel);
  steering_angle_map = steering_angle*42735.042735;
std::cout<<"Steering angle map:"<<steering_angle_map<<std::endl;
  if(steering_angle_map > 20000){
    steering_angle_map = 20000;
  }
  else if(steering_angle_map < -20000){
    steering_angle_map = -20000;
  }
  else{
	steering_angle_map = steering_angle_map;	
}
}

void PathPlanner_Callback(const nav_msgs::Odometry::ConstPtr& msg){
  path_lin_vel = msg->twist.twist.linear.x;
  path_ang_vel = msg->twist.twist.angular.z;
  //std::cout<<"Path linear velocity:"<<path_lin_vel<<std::endl;
  //std::cout<<"Path angular velocity:"<<path_ang_vel<<std::endl;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "golf_controller");
  ros::NodeHandle nh;
  ros::Subscriber odom_sub = nh.subscribe("odom", 1000, PathPlanner_Callback);
  ros::Publisher vel_pub = nh.advertise<std_msgs::Float64>("cmd_accelerator_position", 1000);
  ros::Publisher steer_pub = nh.advertise<std_msgs::Float64>("cmd_steering_angle", 1000);

  ros::Rate rate(10);

  while(nh.ok()){
    ros::spinOnce();

       
    SetLinearVelocity(path_lin_vel);
    SetSteeringWheelPosition(path_ang_vel);

    lin_vel_pub.data = linear_vel;
    ang_vel_pub.data = steering_angle_map;

    vel_pub.publish(lin_vel_pub);
    steer_pub.publish(ang_vel_pub);

    rate.sleep();
  }


  return 0;
}
