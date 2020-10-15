#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/LaserScan.h>
/* 270 values in array (1 beam = value)
*/
void scanner(const sensor_msgs::LaserScan::ConstPtr& scan){
  /* 270 values in array (1 beam = value)
  */
  ROS_INFO("Range: [%.5f]", scan->ranges[0]);


  //ROS_INFO("Angle min: [%.5f]", scan->angle_min);
  //ROS_INFO("Angle max: [%.5f]", scan->angle_max);
  //ROS_INFO("Range max: [%.5f]", scan->range_max);
  //ROS_INFO("Angle inc: [%.5f]", scan->angle_increment);
}
int main(int argc, char** argv){
ros::init(argc, argv, "scanner");
ros::NodeHandle nh;

ros::Subscriber sub=nh.subscribe("/scan",10,scanner);
ros::spin();
return 0;
}
