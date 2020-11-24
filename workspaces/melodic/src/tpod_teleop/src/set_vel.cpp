/*
=============== AUTHORS ===============
Frederik Johannes Christensen (fjch18@student.aau.dk)
20gr560 (5th Semester Robotics AAU)
=======================================
*/

//============= INCLUDES ==============
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <sstream>
#include "geometry_msgs/Twist.h"
#include <iostream>
#include <fstream>
//=====================================

class _set_vel{
public:
  _set_vel(){
    set_vel_pub = n.advertise<std_msgs::Float64>("cmd_accelerator_position", 1000);
  }

  void _publish_vel(){
    msg.data = set_vel_input;
    set_vel_pub.publish(msg);
    ROS_INFO ("set_vel: [%f]", msg.data);
  }

  void _input_vel(){
    std::cin >> set_vel_input;
  };


private:
  ros::Publisher set_vel_pub;
  ros::NodeHandle n;
  double set_vel_input;
  std_msgs::Float64 msg;
};

int main(int argc, char **argv) //Required inits, allows node to take cmds from externally through terminal etc.
{
  ros::init(argc, argv, "set_vel"); //Inits the ros node by name with arguments.
  ROS_INFO("set_vel started");
  ros::NodeHandle n;
  _set_vel _set_vel;
  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    _set_vel._input_vel();
    _set_vel._publish_vel();
    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
