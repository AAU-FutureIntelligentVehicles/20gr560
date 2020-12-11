#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/LaserScan.h>
#include "global.h"
#include <std_msgs/Float64.h>
#include <algorithm>
class tim{
private:
  scan s;
  results r;
  ros::Time newtime;
  ros::NodeHandle nh;
  ros::Publisher pub, pub2;
  ros::Subscriber sub;
  float inf = std::numeric_limits<float>::infinity();
  sensor_msgs::LaserScan msg;
  std_msgs::Float64 speed, brk;
  float current_pos = 0;

public:
void publisher(){
  pub=nh.advertise<sensor_msgs::LaserScan>("scan_rear", 1000);
  pub2=nh.advertise<std_msgs::Float64>("cmd_accelerator_position", 1000);
}
void subscriber(){
  sub= nh.subscribe("scanr",10,&tim::scanner, this);
}
void subscriber2(){
    //sub2= nh.subscribe("cmd_accelerator_position", 10, &tim::acceleration, this); //TODO: Subscribe to cmd topic from Golf Control node
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
    distance(i, current_pos);
  }
}
/*void acceleration(const std_msgs::Float64::ConstPtr& acc){
    current_pos = acc->data;
    //ROS_INFO("Current_pos: [%f]", current_pos);
    //distance(0,current_pos);
} *///TODO: Subscribe to cmd topic from Golf Control node
void distance(int i, float j){
    msg.ranges.resize(180);
    msg.ranges[i] = s.ranges[i];
    float* min_elem;
    float speed_setpoint = 0.8; // Manually setting the cmd accelerator position of the car
    min_elem = std::min_element(s.ranges + 0, s.ranges + 180); // Finding the minimal value of the scan array
    if(*min_elem > 8 && *min_elem != inf){
        std::cout<<"Data: "<<*min_elem<<std::endl;
        speed.data = speed_setpoint * 1;
        std::cout<<"speed:  "<<speed.data<<std::endl;
    }
    else if(*min_elem < 8 && *min_elem > 6 && *min_elem != inf){  //4,3
            std::cout<<"*Data: "<<*min_elem<<std::endl;
            speed.data = speed_setpoint * 0.8;
            std::cout<<"speed:  "<<speed.data<<std::endl;
        }
    else if(*min_elem < 6 && *min_elem > 4 && *min_elem != inf){ //3, 2
            std::cout<<"**Data: "<<*min_elem<<std::endl;
             speed.data = speed_setpoint * 0.6;
        std::cout<<"speed:  "<<speed.data<<std::endl;
        }
    else if(*min_elem < 4 && *min_elem > 2 && *min_elem != inf){ //2, 1
            std::cout<<"***Data: "<<*min_elem<<std::endl;
            speed.data = speed_setpoint * 0.4;
        std::cout<<"speed:  "<<speed.data<<std::endl;
        }
    else if(*min_elem < 2 && *min_elem > 1 && *min_elem != inf){ //2, 1
        std::cout<<"****Data: "<<*min_elem<<std::endl;
        speed.data = speed_setpoint * 0.2;
        std::cout<<"speed:  "<<speed.data<<std::endl;
    }
    else if(*min_elem < 1 && *min_elem > 0.05 && *min_elem != inf){ //2, 1
        std::cout<<"*****Data: "<<*min_elem<<std::endl;
        speed.data = speed_setpoint * 0;
        std::cout<<"speed:  "<<speed.data<<std::endl;
    }
    pub.publish(msg);
    pub2.publish(speed);
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
