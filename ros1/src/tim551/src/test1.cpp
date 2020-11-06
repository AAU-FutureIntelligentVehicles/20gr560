#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/LaserScan.h>

class test{
private:
  ros::NodeHandle nh;
  ros::Subscriber sub;
public:
  void subscriber(){
    sub=nh.subscribe("average_scan",10,&test::callback, this);
  }
  void callback(const sensor_msgs::LaserScan::ConstPtr& marker){
    ROS_INFO("Ranges: [%.5f]", marker->ranges[0]);
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
