#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/LaserScan.h>
#include <queue>
#include <limits>
#include "global.h"
#include <tf/transform_broadcaster.h>
#include <iostream>
ros::Time newtime;

class tim{
private:
  scan s;
  results r;
  ros::NodeHandle nh;
  ros::Publisher pub;
  ros::Subscriber sub;
  std::queue<float> q;
  float inf = std::numeric_limits<float>::infinity();
  sensor_msgs::LaserScan msg;
public:

void publisher(){
  pub=nh.advertise<sensor_msgs::LaserScan>("scan_rear", 1000);
}

void subscriber(){
  sub= nh.subscribe("scanr",10,&tim::scanner, this);
}
void scanner(const sensor_msgs::LaserScan::ConstPtr& scan){
    //ROS_INFO("Scan time: [%]", scan->header.stamp);
    newtime = scan->header.stamp;
    msg.header.frame_id = "laser_rear";
    msg.header.stamp = newtime;
    msg.angle_min = -1.570796;
    msg.angle_max = -1.570796;
    msg.angle_increment= 0.01745329238474369;
    msg.time_increment= 0.00018518499564379454;
    msg.scan_time = 0.06666667014360428;
    msg.range_min = 0.05000000074505806;
    msg.range_max = 8;

  for(int i=0;i<(scan->ranges.size());i++){
            if(!(scan->ranges.empty()))
            s.ranges[i] = scan->ranges[i];
          }
  for (int i = 0; i <(scan->ranges.size()); i++) {
    distance(i);
  }
}

void distance(int i){
    msg.ranges.resize(180);
    msg.ranges[i] = s.ranges[i];
    if(s.ranges[i] > s.limit && s.ranges[i] != inf){
        pub.publish(msg);
    }
    else{
  //      ROS_ERROR("***Angle is too close***");
       // ros::shutdown();
    }
  //  ROS_INFO("Ranges: [%.5f]", msg.ranges[i]);

}
};
int main(int argc, char** argv){
ros::init(argc, argv, "scanner");
tim T;
T.subscriber();
T.publisher();
ros::spin();
return 0;
}



//Old code for average calculation//
/*
  msg.header.frame_id = "laser_mount_link";
  msg.header.stamp = ros::Time();
  msg.angle_min = -1.570796;
  msg.angle_max = -1.570796;
  msg.angle_increment= 0.01745329238474369;
  msg.time_increment= 0.00018518499564379454;
  msg.scan_time = 0.06666667014360428;
  msg.range_min = 0.05000000074505806;
  msg.range_max = 8;
if (s.ranges[i] > s.limit && s.ranges[i] != inf){
  q.push(s.ranges[i]);
  r.sum += q.back();
    if(q.size()>s.len){
      msg.ranges.resize(180); // resizing the cpp array to ros array
      r.avg = r.sum/15;
      msg.ranges[i] = r.avg;
      pub.publish(msg);
      r.lrange[i] = r.avg;
      ROS_INFO("Average: [%.5f]", r.lrange[i]);
      r.sum -= q.front();
      q.pop();
    }
    }
else{
  ROS_ERROR("***Angle is too close***");
} */
