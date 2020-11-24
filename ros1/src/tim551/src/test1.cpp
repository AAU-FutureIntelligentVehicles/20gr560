#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float64.h>

class test{
private:
  ros::NodeHandle nh;
  ros::Subscriber sub, sub1;
public:
  void subscriber(){
    sub=nh.subscribe("velocity",10,&test::callback, this);
    sub1=nh.subscribe("break", 10, &test::callback1, this);
  }
  void callback(const std_msgs::Float64::ConstPtr& speed){
    ROS_INFO("Velocity: [%f]", speed->data);
  }
  void callback1(const std_msgs::Float64::ConstPtr& brk){
      ROS_INFO("Break: [%f]", brk->data);
  }
};

int main(int argc, char** argv){
  ros::init(argc, argv, "test");
  //ros::NodeHandle nh;
  test T;
  T.subscriber();
  //ros::Subscriber sub=nh.subscribe("floater",10,&bim::halo, &T);
  ros::spin();
  return 0;
}
