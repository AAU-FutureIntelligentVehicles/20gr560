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
#include <ackermann_msgs/AckermannDriveStamped.h>

double linear_vel = 0;
double wheel_base = 1.64;
double steering_pos;
double path_lin_vel, path_ang = 0;
double steering_resolution = 20000;
double max_steering_angle = 0.617322956;
double min_vel = 0.5;
double steering_resolution_map = max_steering_angle/steering_resolution;

std_msgs::Float64 lin_vel_pub;
std_msgs::Float64 ang_pub;

void SetLinearVelocity(double desired_lin_vel){
  linear_vel = 0.6252*abs(desired_lin_vel) + 0.4173; //taking abs() because we cannot switch directions with + and -
  if(linear_vel < min_vel){
    linear_vel = min_vel;
  }
}

void SetSteeringWheelPosition(double desired_ang){
  steering_pos = desired_ang/steering_resolution_map;
  if(steering_pos > steering_resolution){
    steering_pos = steering_resolution;
  }
  else if(steering_pos < -steering_resolution){
    steering_pos = -steering_resolution;
  }
  else{
    steering_pos = steering_pos;
  }
}


void cmdvel_Callback(const geometry_msgs::Twist::ConstPtr& msg){
  path_lin_vel = double(msg->linear.x);
  path_ang = double(msg->angular.z);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "golf_controller");
  ros::NodeHandle nh;
  ros::Subscriber cmd_vel_sub = nh.subscribe("cmd_vel", 1, cmdvel_Callback);
  ros::Publisher vel_pub = nh.advertise<std_msgs::Float64>("cmd_accelerator_position", 1000);
  ros::Publisher steer_pub = nh.advertise<std_msgs::Float64>("cmd_steering_angle", 1000);

  ros::Rate rate(5);

  while(nh.ok()){
    ros::spinOnce();

    if(path_lin_vel){
        SetLinearVelocity(path_lin_vel);
    }
    SetSteeringWheelPosition(path_ang);

    lin_vel_pub.data = linear_vel;
    ang_pub.data = steering_pos;

    vel_pub.publish(lin_vel_pub);
    steer_pub.publish(ang_pub);

    rate.sleep();
  }


  return 0;
}
