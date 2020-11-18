#include <memory>

#include "rclcpp/rclcpp.hpp"

#include <sensor_msgs/msg/laser_scan.hpp>
using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
    MinimalSubscriber()
            : Node("minimal_subscriber")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
                "averages", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }

private:
    void topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan) const
    {
        RCLCPP_INFO(this->get_logger(), "Average: [%.5f]", scan->ranges[179]);
    }
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalSubscriber>());
    rclcpp::shutdown();
    return 0;
}