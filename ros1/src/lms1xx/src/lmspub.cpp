#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Header.h>
#include "std_msgs/String.h"
//#include <visualization_msgs/Marker.h>

class bim{
private:
    ros::NodeHandle nh;
    ros::Subscriber sub;
    ros::Publisher vis_pub;
  //  sensor_msgs::LaserScan marker;
public:
    void subscriber(){
        sub=nh.subscribe("map",10,&bim::scanner, this);
    }
    void scanner(const nav_msgs::OccupancyGrid::ConstPtr& occ){
        //Testing occupancy grid of hector_slam
        std_msgs::Header header = occ->header;
        nav_msgs::MapMetaData info = occ->info;
        ROS_INFO("Header: [%s]", header.frame_id.c_str());
        ROS_INFO("Width: [%d] ", info.width);
        ROS_INFO("Height: [%d]", info.height);

        ROS_INFO("Got map %d %d", info.width, info.height);
        /*
        if (debug) ROS_INFO("\tbreakpoint - set_map 1");
           //This recieves an OccupancyGrid pointer and makes it global to be used in
           //"goodPoint", besides that, it renders an updated 2D image of the map.
           mMap = m;
           std::string  mapName = "/home/vdr/ros1/mapperoni.pgm";
           ROS_INFO("\nNow I will draw the map");
           FILE* mapFile = fopen(mapName.c_str(), "w");
           if (!mapFile){
               ROS_ERROR("\nSorry!!!....I could not save the map :(");
               return;
           }
           fprintf(mapFile, "P5\n# Mars map, Space program: C2-20, Mission: RobP1-18 %.3f m/pix\n%d %d\n255\n",
                   mMap->info.resolution, mMap->info.width, mMap->info.height);
           for(unsigned int y = 0; y < mMap->info.height; y++){
               for(unsigned int x = 0; x < mMap->info.width; x++){
                   unsigned int i = x + (mMap->info.height - y - 1) * mMap->info.width;
                   if (mMap->data[i] == 0){
                       fputc(254, mapFile);
                   } else if (mMap->data[i] == +100){
                       fputc(000, mapFile);
                   } else {
                       fputc(205, mapFile);
                   }
               }
           }
           fclose(mapFile);
           ROS_INFO("\nI saved the map ^_^"); */
    }

};
int main(int argc, char** argv){
    ros::init(argc, argv, "lmsscan");
    bim T;
    ROS_INFO("Publishing to RVIZ under the topic /average_scan");
    T.subscriber();
   // T.publisher();
    ros::spin();
    return 0;
}