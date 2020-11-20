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
double odometry_vel = 0;

class _key_teleop{
public:
  //Constructor
  _key_teleop() {
    _sub_key_teleop = n.subscribe("key_vel", 10, &_key_teleop::KeyCallback, this);
    set_vel_sub = n.subscribe("tpod_set_vel", 1, &_key_teleop::set_velCallback, this);
    odometry_sub = n.subscribe("odometry", 1, &_key_teleop::odometryCallback, this);
  }

  //Callbacks
  void KeyCallback(const geometry_msgs::Twist::ConstPtr& msg){
    ROS_INFO("Speed: [%f], Turn: [%f]", msg->linear.x, msg->angular.z);
    current_vel = msg->linear.x;
    current_turn = msg->angular.z;
    //set_vel = current_vel;
  }

  void set_velCallback(const std_msgs::Float64::ConstPtr& msg){
    set_vel = msg->data;
  }

  void odometryCallback(const std_msgs::Float64::ConstPtr& msg){
    odometry_vel = msg->data;
  };


private:
  ros::NodeHandle n;
  ros::Publisher set_vel_pub;
  ros::Subscriber _sub_key_teleop,
                  set_vel_sub,
                  odometry_sub;
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
  myFile << "odometry_vel\n";

  while (ros::ok())
  {

    myFile << set_vel;
    myFile << ",";
    myFile << odometry_vel << '\n';
    ros::spinOnce();
    loop_rate.sleep();
  }


  ROS_INFO("Closing and saving log file");
  myFile.close();
  return 0;
}
