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
  //float sum(0);
  std::queue<float> q;
public:
  void scanner(const sensor_msgs::LaserScan::ConstPtr& scan){
    //ROS_INFO("Original: [%.5f]",scan->ranges[179]);
    s.ranges = scan->ranges[179];
    ROS_INFO("Original: [%.5f]",s.ranges);
    distance();
  //  sum += q.back();
/*    if(q.size()>len){
      sum -= q.front();
      sum += q.back();
    }
    ROS_INFO("Current SUM: [%.5f]", sum);*/
  }

  void distance(){
    float avg;
      if (s.ranges > limit){
        q.push(s.ranges);
        sum += q.back();
        ROS_INFO("Current SUM: [%.5f]", sum);
        //ROS_INFO("New back: [%.5f]", q.back());
        ROS_INFO("%ld", q.size());
          if(q.size()>len){
          avg = sum/15;
          ROS_INFO("First element of the queue: [%.5f]", q.front());
          ROS_INFO("Last element of the queue: [%.5f]", q.back());
          ROS_INFO("SUM TOTAL: [%.5f]", sum);
          ROS_INFO("Average: [%.5f]", avg);
          sum -= q.front();
          //sum = 0;
          q.pop();
          //ROS_INFO("[%.5f]", q.pop());
          //ros::shutdown();
        }
      }
    //  ROS_INFO("%ld", q.size());
      else{
        ROS_INFO("It is too close!");
        //q.front() = 0;
        //ROS_INFO("[%.5f]",q.front());
    }
  /*    while(!q.empty()){
      ROS_INFO("Ranges: [%.5f]", q.front());
      sum += q.front();
      avg = sum/15;
      q.pop();
  }*/
//  ROS_INFO("READING: [%.5f]", s.ranges);
}
};


int main(int argc, char** argv){
ros::init(argc, argv, "scanner");
ros::NodeHandle nh;
tim551 T;
ros::Subscriber sub=nh.subscribe("/scan",10,&tim551::scanner, &T);
T.distance();


//tim TC;
//ros::Subscriber sub=nh.subscribe("/scan",10,scanner);
//TC.scanner();
ros::spin();
return 0;
}
