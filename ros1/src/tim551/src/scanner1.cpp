#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/LaserScan.h>
#include <queue>
#include <limits>
#include "global.h"

// 270 values in array (1 beam = value)
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
//(scan->ranges.size())
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
    //ROS_ERROR("Angle: [%d] deg is too close!", j);
  }



//************ FOR 5 ANGLES ONLY****************//
/*  if(i == 0){
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
  }*/


/*
  if (s.ranges[i] > s.limit && s.ranges[i] != inf){
    q.push(s.ranges[i]);
    r.sum += q.back();
      if(q.size()>s.len){
        msg.ranges.resize(5); // resizing the cpp array to ros array
        r.avg = r.sum/15;
        msg.ranges[i] = r.avg;
        pub.publish(msg);
        ROS_INFO("Angle: [%d] deg", j);
        r.lrange[i] = r.avg;
        ROS_INFO("Average: [%.5f]", r.lrange[i]);
        r.sum -= q.front();
        q.pop();
        }
      }
  else{
    ROS_ERROR("***CRITICAL ERROR***");
    ROS_ERROR("Angle: [%d] deg is too close!", j);
  } */
//****************************************************


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
