/*============= AUTHORS ===============
Frederik Johannes Christensen (fjch18@student.aau.dk)
20gr560 (5th Semester Robotics AAU)
=====================================*/


//============= INCLUDES ==============
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <sstream>
#include "geometry_msgs/Twist.h"
#include <iostream>
#include <fstream>
//=====================================
double set_vel = 0;

class _key_teleop{
public:
  //Constructor
  _key_teleop() {
    _sub_key_teleop = n.subscribe("key_vel", 10, &_key_teleop::KeyCallback, this);
    set_vel_pub = n.advertise<std_msgs::Float64>("tpod_set_vel", 1000);
  }

  //Callbacks
  void KeyCallback(const geometry_msgs::Twist::ConstPtr& msg){
    ROS_INFO("Speed: [%f], Turn: [%f]", msg->linear.x, msg->angular.z);
    current_vel = msg->linear.x;
    current_turn = msg->angular.z;
    set_vel = current_vel;
  }

  void _log_vel(){

  }

  void _log_turn(){

  };


private:
  ros::NodeHandle n;
  ros::Publisher set_vel_pub;
  ros::Subscriber _sub_key_teleop;
  double current_vel = 0;
  double current_turn = 0;
};




int main(int argc, char **argv) //Required inits, allows node to take cmds from externally through terminal etc.
{
  ros::init(argc, argv, "tpod_teleop"); //Inits the ros node by name with arguments.
  ROS_INFO("tpod_teleop initiated");
  ros::NodeHandle n;
  _key_teleop _key_teleop;
  ros::Rate loop_rate(10);
  std::ofstream myFile("vel_turn_acc_log.csv");
  myFile << "set_vel,";
  myFile << "odometry_feedback\n";

  while (ros::ok())
  {

    myFile << set_vel;
    myFile << ",";
    myFile << "feedback\n";
    ros::spinOnce();
    loop_rate.sleep();
  }


  ROS_INFO("Closing and saving log file");
  myFile.close();
  return 0;
}
