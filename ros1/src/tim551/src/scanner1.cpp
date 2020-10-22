#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/LaserScan.h>
#include <queue>
#include "global.h"

// 270 values in array (1 beam = value)
class tim{
private:
  scan s;
  float sum;
  results r;
  float avg;
  float lrange[4];
  ros::NodeHandle nh;
  ros::Publisher pub;
  ros::Subscriber sub;
  std::queue<float> q;
  sensor_msgs::LaserScan msg;
public:

void publisher(){
  pub=nh.advertise<sensor_msgs::LaserScan>("floater", 1000);
}

void subscriber(){
  sub= nh.subscribe("/scan",10,&tim::scanner, this);
}
void scanner(const sensor_msgs::LaserScan::ConstPtr& scan){
  s.ranges[0] = scan->ranges[179];
  s.ranges[1] = scan->ranges[178];
  s.ranges[2] = scan->ranges[177];
  s.ranges[3] = scan->ranges[176];
  s.ranges[4] = scan->ranges[175];
  distance(0);
  distance(1);
  distance(2);
  distance(3);
  distance(4);
}

void distance(int i){
  int j;
  if(i == 0){
    j = 180;
  }
  else if(i == 1){
    j = 179;
  }
  else if(i == 2){
    j = 178;
  }
  else if(i == 3){
    j = 177;
  }
  else if(i == 4){
    j = 176;
  }

  if (s.ranges[i] > s.limit){
    q.push(s.ranges[i]);
    sum += q.back();
      if(q.size()>s.len){
        msg.ranges.resize(5); // resizing the cpp array to ros array
        avg = sum/15;
        msg.ranges[i] = avg;
        pub.publish(msg);
        ROS_INFO("Angle: [%d] deg", j);
        lrange[i] = avg;
        ROS_INFO("Average: [%.5f]", lrange[i]);
        sum -= q.front();
        q.pop();
      }
      }
  else{
    ROS_ERROR("***CRITICAL ERROR***");
    ROS_ERROR("Angle: [%d] deg is too close!", j);
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



//ROS_INFO("First element of the queue: [%.5f]", q.front());
//ROS_INFO("Last element of the queue: [%.5f]", q.back());
//  ROS_INFO("SUM TOTAL: [%.5f]", sum);
//ROS_INFO("Current SUM: [%.5f]", sum);
//ROS_INFO("%ld", q.size());
