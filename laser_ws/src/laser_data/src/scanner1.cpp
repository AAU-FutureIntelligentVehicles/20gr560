#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/LaserScan.h>
#include <queue>
#include <limits>
#include "global.h"
#include <std_msgs/Float64.h>

class tim{
private:
  scan s;
  results r;
  ros::Time newtime;
  ros::NodeHandle nh;
  ros::Publisher pub, pub2, pub3;
  ros::Subscriber sub;
  std::queue<float> q;
  float inf = std::numeric_limits<float>::infinity();
  sensor_msgs::LaserScan msg;
  std_msgs::Float64 speed, brk;

public:

void publisher(){
  pub=nh.advertise<sensor_msgs::LaserScan>("scan_rear", 1000);
  pub2=nh.advertise<std_msgs::Float64>("velocity", 1000);
  pub3=nh.advertise<std_msgs::Float64>("break", 1000);
}

void subscriber(){
  sub= nh.subscribe("scanr",10,&tim::scanner, this);
}
void scanner(const sensor_msgs::LaserScan::ConstPtr& scan){
    newtime = scan->header.stamp;
    msg.header.frame_id = "laser_rear";
    msg.header.stamp = newtime;
    msg.header.seq = scan->header.seq;
    msg.angle_min = scan->angle_min;
    msg.angle_max = scan->angle_max;
    msg.angle_increment= scan->angle_increment;
    msg.time_increment= scan->time_increment;
    msg.scan_time = scan->scan_time;
    msg.range_min = scan->range_min;
    msg.range_max = scan->range_max;

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
        speed.data = 1;
        pub2.publish(speed);
        pub.publish(msg);
        //ROS_INFO("Angle:[%.5f]", s.ranges[i]);
    }
    else{
  //      ROS_ERROR("***Angle is too close***");
        if(s.ranges[i] < 0.5 && s.ranges[i] > 0.3 && s.ranges[i] != inf){  //9, 7
            speed.data = 0.8;
            pub2.publish(speed);
        }
        else if(s.ranges[i] < 0.3 && s.ranges[i] > 0.1 && s.ranges[i] != inf){ //7, 5
             speed.data = 0.6;
            pub2.publish(speed);
        }
        else if(s.ranges[i] < 0.1 && s.ranges[i] > 0.0 && s.ranges[i] != inf){ //5, 3
            speed.data = 0.4;
            brk.data = 3;
            pub2.publish(speed);
            pub3.publish(brk);
        }
    }

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
    } */