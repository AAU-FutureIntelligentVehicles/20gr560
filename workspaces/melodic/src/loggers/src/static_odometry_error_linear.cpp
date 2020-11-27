/*============= AUTHORS ===============
Frederik Johannes Christensen (fjch18@student.aau.dk)
20gr560 (5th Semester Robotics AAU)
=====================================*/


//============= INCLUDES ==============
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <sstream>
#include "nav_msgs/Odometry.h"
#include <iostream>
#include <fstream>
//=====================================
double x_pos, y_pos, dist_left_wheel, velocity = 0;
ros::Time timestamp;

class logger{
public:
  //Constructor
  logger(){
    odom_sub = nh.subscribe("odom", 50, &logger::Odom_Callback, this);
    dist_sub = nh.subscribe("dist", 1, &logger::Dist_Callback, this);
  }

  //Callbacks
  void Dist_Callback(const std_msgs::Float64::ConstPtr& msg){
    dist_left_wheel = msg->data;
  }
  void Odom_Callback(const nav_msgs::Odometry::ConstPtr& msg){
    x_pos = msg->pose.pose.position.x;
    y_pos = msg->pose.pose.position.y;
    timestamp = msg->header.stamp;
    velocity = msg->twist.twist.linear.x;
  };

private:
  ros::NodeHandle nh;
  ros::Subscriber odom_sub,
                  dist_sub;
};




int main(int argc, char **argv) //Required inits, allows node to take cmds from externally through terminal etc.
{
  ros::init(argc, argv, "static_odometry_error_linear_logger"); //Inits the ros node by name with arguments.
  ros::NodeHandle n;
  logger logger;
  ros::Rate loop_rate(10);
  std::ofstream myFile("static_odometry_error_linear_log.csv");
  myFile << "dist_left_wheel, x_pos, y_pos, timestamp, velocity" << '\n';

  while (ros::ok())
  {
    system("clear");
    myFile << dist_left_wheel << "," << x_pos << "," << y_pos << "," << timestamp << "," << velocity << '\n';
    std::cout << "dist_left_wheel: " << dist_left_wheel << "  x_pos: " << x_pos << "  y_pos: " << y_pos << "  timestamp: " <<  timestamp << "  velocity: " << velocity << '\n';
    ros::spinOnce();
    loop_rate.sleep();
  }


  ROS_INFO("Closing and saving log file");
  myFile.close();
  return 0;
}
