/*
=============== AUTHORS ===============
Frederik Johannes Christensen (fjch18@student.aau.dk)
20gr560 (5th Semester Robotics AAU)
=======================================
*/

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <sstream>


int main(int argc, char **argv)
{

  ros::init(argc, argv, "tpod_teleop");


  ros::NodeHandle n;


  ros::Publisher set_vel_pub = n.advertise<std_msgs::Float64>("tpod_set_vel", 1000);

  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {

    std_msgs::Float64 set_vel_msg;

    std::stringstream ss;
    ss << "Set velocity: " << count;
    set_vel_msg.data = double(count);

    //ROS_INFO("%s", set_vel.data.c_str());

    set_vel_pub.publish(set_vel_msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
