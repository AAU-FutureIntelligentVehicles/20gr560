#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>

class bim{
private:
  ros::NodeHandle nh;
  ros::Subscriber sub;
  ros::Publisher vis_pub;
  sensor_msgs::LaserScan marker;
  float average[4];
public:
  void subscriber(){
    sub=nh.subscribe("averages",10,&bim::halo, this);
  }
  void publisher(){
    vis_pub = nh.advertise<sensor_msgs::LaserScan>("average_scan", 1000);
  }
  void halo(const sensor_msgs::LaserScan::ConstPtr& msg){
    average[0] = msg->ranges[0];
    average[1] = msg->ranges[1];
    average[2] = msg->ranges[2];
    average[3] = msg->ranges[3];
    average[4] = msg->ranges[4];
    visualization();
}
  void visualization(){
    marker.ranges.resize(5);
    marker.header.frame_id = "scanneroni";
    marker.header.stamp = ros::Time();
    marker.angle_min = -1.570796;
    marker.angle_max = -1.570796;
    marker.angle_increment= 0.01745329238474369;
    marker.time_increment= 0.00018518499564379454;
    marker.scan_time = 0.06666667014360428;
    marker.range_min = 0.05000000074505806;
    marker.range_max = 6;

    marker.ranges[0] = average[0];
    marker.ranges[1] = average[1];
    marker.ranges[2] = average[2];
    marker.ranges[3] = average[3];
    marker.ranges[4] = average[4];
    marker.intensities[0];
    vis_pub.publish(marker);
}

};
int main(int argc, char** argv){
  ros::init(argc, argv, "avgscan");
  bim T;
  ROS_INFO("Publishing to RVIZ under the topic /average_scan");
  T.subscriber();
  T.publisher();
  ros::spin();
  return 0;
}
