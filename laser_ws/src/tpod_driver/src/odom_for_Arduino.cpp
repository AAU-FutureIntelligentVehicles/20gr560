#include "ros/ros.h"
#include "std_msgs/String.h"
#include <tf/transform_broadcaster.h>
#include "serial/serial.h"
#include <sstream>
#include <iostream>   // std::cout
#include <cstring>     // std::string, std::stof
#include <math.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);//publisher på topic "chatter"
  //ros::Rate loop_rate(10);

  static tf::TransformBroadcaster br;

  int count = 0;
  double left, right, left_old, right_old = 0;
  double delta_left, delta_right = 0;
  double yaw = 0;
  double x, y, delta_x, delta_y = 0;

  serial::Serial my_serial("/dev/ttyACM0", 115200, serial::Timeout::simpleTimeout(1000));
  my_serial.flush();


  while (ros::ok()) {
	
// === Read from serial ===  

    if(my_serial.isOpen()) {
   		//std::cout << "Serial port open" << std::endl; //Opens serial port
	    std::string data = my_serial.readline(); //Puts data from seriel port in to string "data"
		
		std::string komma = ","; // Define "komma"
		int pos_komma = data.find(komma);
	
		std::string string_left = data.substr(0, pos_komma); // Grab first string.
		std::string string_right = data.substr(pos_komma+1, data.find("NULL")); // Grab second string.

   		left = atoi(string_left.c_str()); // Convert from string to int
		right = atoi(string_right.c_str());
		}
	
// === Do the math ===
	
	left = left * 0.0034625; // convert from tick to meter
    right = right * 0.0034625; 
    
    delta_left = (left - left_old); // Difference in movement since last iteration
	delta_right = (right - right_old);

    left_old = left; // save value from this iteration
	right_old = right;

   	yaw = (right - left)/0.98; // Calculate yaw (0.98 = wheelbase)

	delta_x = ((delta_left + delta_right)/2) * cos(yaw); // calculate difference in x movement
	delta_y = ((delta_left + delta_right)/2) * sin(yaw); // calculate difference in y movement
	
	x = x + delta_x; // calculate accumulated x movement
	y = y + delta_y; // calculate accumulated y movement

std::cout<<left<<" "<<right<<" "<<yaw<<" "<<x<<" "<<y<<std::endl;
	
// === Broadcast transformation ===

    tf::Transform transform;
    transform.setOrigin( tf::Vector3(x, y, 0.22) ); //Sæt X og Y her. (Ingen bevægelse i Z-retning.)
    tf::Quaternion q;
    q.setRPY(0, 0, yaw); //Sæt Roll, Pitch, Yaw her. (Kun Yaw er aktuelt.) (Radian)
  
	transform.setRotation(q);

	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "rear_axle"));

//=======================================================
/*
    std_msgs::String msg;

    std::stringstream ss;
    ss << "Im running " << count;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

   
    chatter_pub.publish(msg);
*/
    ros::spinOnce();

    //loop_rate.sleep();
    ++count;
  }


  return 0;
}

