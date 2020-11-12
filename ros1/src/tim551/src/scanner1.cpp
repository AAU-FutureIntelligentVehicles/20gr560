#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/LaserScan.h>
#include <queue>
#include <limits>
#include "global.h"


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
  pub=nh.advertise<sensor_msgs::LaserScan>("averages", 1000);
}

void subscriber(){
  sub= nh.subscribe("scan_rear",10,&tim::scanner, this);
}
void scanner(const sensor_msgs::LaserScan::ConstPtr& scan){
  for(int i=0;i<(scan->ranges.size());i++){
            if(!(scan->ranges.empty()))
            s.ranges[i] = scan->ranges[i];
          }
  for (int i = 0; i <(scan->ranges.size()); i++) {
    distance(i);
  }
}

void distance(int i){
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
