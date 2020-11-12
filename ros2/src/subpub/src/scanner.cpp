#include <memory>
#include <chrono>
#include <functional>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

using std::placeholders::_1;

class Poggers : public rclcpp::Node
{
public:
    Poggers()
            : Node("poggers")
    {
        RCLCPP_INFO(this->get_logger(), "Welcome to poggers!");
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
                "averages", 10, std::bind(&Poggers::topic_callback, this, _1));
        publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan2", 15);
    }


private:
    void topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan) const {
        auto marker = sensor_msgs::msg::LaserScan();
        marker.ranges.resize(180);
        marker.header.frame_id = "ros2scan";
        marker.header.stamp.sec = 0;
        marker.angle_min = -1.570796;
        marker.angle_max = -1.570796;
        marker.angle_increment= 0.01745329238474369;
        marker.time_increment= 0.00018518499564379454;
        marker.scan_time = 0.06666667014360428;
        marker.range_min = 0.05000000074505806;
        marker.range_max = 6;
        for(unsigned int i=0;i<scan->ranges.size();i++){
            if(!(scan->ranges.empty()))
                marker.ranges[i] = scan->ranges[i];
        }
        marker.intensities[0];
        publisher_->publish(marker);
    }
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Poggers>());
    rclcpp::shutdown();
    return 0;
}