/*
=============== AUTHORS ===============
Frederik Johannes Christensen (fjch18@student.aau.dk)
=======================================
*/

//============= INCLUDES ==============
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int64.h"
#include <sstream>
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include <iostream>
#include <fstream>
//=====================================
double printdata[6];
int printint[2];

class watchdog{
public:
  watchdog(){
    _steering_sub = nh.subscribe("cmd_steering_angle", 1, &watchdog::steerCallback, this);
    _accelerator_sub = nh.subscribe("cmd_accelerator_position", 1, &watchdog::acceleratorCallback, this);
    _brake_sub = nh.subscribe("set_brake_position", 1, &watchdog::brakeCallback, this);
    _ticks_left = nh.subscribe("ticks_left", 1, &watchdog::ticks_leftCallback, this);
    _ticks_right = nh.subscribe("ticks_right", 1, &watchdog::ticks_rightCallback, this);
    _odom_sub = nh.subscribe("odom", 10, &watchdog::Odom_Callback, this);
  }

  void steerCallback(const std_msgs::Float64::ConstPtr& new_setpoint){
    printdata[0] = new_setpoint->data;
  }

  void acceleratorCallback(const std_msgs::Float64::ConstPtr& new_setpoint){
    printdata[1] = new_setpoint->data;
  }

  void brakeCallback(const std_msgs::Float64::ConstPtr& new_setpoint){
    printdata[2] = new_setpoint->data;
  }

  void ticks_leftCallback(const std_msgs::Int64::ConstPtr& new_setpoint){
    printdata[3] = double(new_setpoint->data);
  }

  void ticks_rightCallback(const std_msgs::Int64::ConstPtr& new_setpoint){
    printdata[4] = double(new_setpoint->data);
  }

  void Odom_Callback(const nav_msgs::Odometry::ConstPtr& msg){
    printdata[5] = msg->twist.twist.linear.x;
  }

private:
  ros::NodeHandle nh;
  ros::Subscriber _steering_sub,
                  _accelerator_sub,
                  _brake_sub,
                  _ticks_right,
                  _ticks_left,
                  _odom_sub;

  //ros::Publisher;
};



int main(int argc, char **argv) //Required inits, allows node to take cmds from externally through terminal etc.
{
  ros::init(argc, argv, "watchdog"); //Inits the ros node by name with arguments.
  ROS_INFO("Watchdog now running");
  ros::NodeHandle n;
  watchdog watchdog;
  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    system("clear");
    std::cout << "Steering: " << printdata[0] <<'\n';
    std::cout << "Accelerator: " << printdata[1] <<'\n';
    std::cout << "Brake: " << printdata[2] << '\n';
    std::cout << "Ticks left: " << printdata[3] << '\n';
    std::cout << "Ticks right: " << printdata[4] << '\n';
    std::cout << "Vel: " << printdata[5] << '\n';
    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
