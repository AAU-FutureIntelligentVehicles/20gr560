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
//=====================================

class _get_key_teleop{
public:
  //Constructor
  _get_key_teleop() {
    ros::Subscriber _sub_key_teleop = n.subscribe("key_vel", 10, &_get_key_teleop::KeyCallback, this);
    ros::Publisher set_vel_pub = n.advertise<std_msgs::Float64>("tpod_set_vel", 1000);
    ROS_INFO("_get_key_teleop class was constructed");
  }
  
  //Callbacks
  void KeyCallback(const geometry_msgs::Twist::ConstPtr& msg);
private:
  ros::NodeHandle n;
};

void _get_key_teleop::KeyCallback(const geometry_msgs::Twist::ConstPtr& msg){
  ROS_INFO("CALLBACK RAN");
}


int main(int argc, char **argv) //Required inits, allows node to take cmds from externally through terminal etc.
{
  ros::init(argc, argv, "tpod_teleop"); //Inits the ros node by name with arguments.
  ROS_INFO("tpod_teleop initiated");
  ros::NodeHandle n;
  _get_key_teleop get_key_teleop;
  ros::Rate loop_rate(10);


  while (ros::ok())
  {

    std_msgs::Float64 set_vel_msg;
    set_vel_msg.data = double(0.1);

    //ROS_INFO("%s", set_vel.data.c_str());

    //set_vel_pub.publish(set_vel_msg);
    //get_key_teleop.set_vel_pub.publish(double(0.0));

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}
