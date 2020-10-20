#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/LaserScan.h>
#include <queue>
#include <vector>
#include "global.h"
#include <vector>
#include <numeric>
#include <string>
#include <functional>
// 270 values in array (1 beam = value)

class tim551{
private:
  scan s;
  float sum;
  int i = 0;
  float one, two;
  float limit = 0.3;
  float len = 14;
  std::queue<float> q;
public:
  void scanner(const sensor_msgs::LaserScan::ConstPtr& scan){
    s.ranges = scan->ranges[179];
    ROS_INFO("Original: [%.5f]",s.ranges);
    distance();
  }

  void distance(){
    float avg;
      if (s.ranges > limit){
        q.push(s.ranges);
        sum += q.back();
        ROS_INFO("Current SUM: [%.5f]", sum);
        ROS_INFO("%ld", q.size());
          if(q.size()>len){
          avg = sum/15;
          ROS_INFO("First element of the queue: [%.5f]", q.front());
          ROS_INFO("Last element of the queue: [%.5f]", q.back());
          ROS_INFO("SUM TOTAL: [%.5f]", sum);
          ROS_INFO("Average: [%.5f]", avg);
          sum -= q.front();
          q.pop();
        }
      }
      else{
        ROS_INFO("It is too close!");
    }
}
};


int main(int argc, char** argv){
ros::init(argc, argv, "scanner");
ros::NodeHandle nh;
tim551 T;
ros::Subscriber sub=nh.subscribe("/scan",10,&tim551::scanner, &T);
T.distance();

ros::spin();
return 0;
}
