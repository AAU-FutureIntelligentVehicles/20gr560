/*
T.Pod Driver

This driver will _connect a ROS network to the embedded system on the T.Pod
golfcart.


Copyright (c) 2017 Karl D. Hansen - Aalborg University

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include <stdexcept>
#include <iostream>
#include <mutex>
#include <thread>

#include <boost/asio.hpp>
#include <boost/regex.hpp>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int64.h>

#include "json.hpp"

// json is under nlohman ns
using namespace nlohmann;


class TPodDriver
{
public:
    /// Construct driver object from a ROS parameter
    ///
    /// @param[in] ip_param  The name of the parameter on the ROS parameter server
    ///                      that holds the address string.
    TPodDriver(std::string ip_param)
    {
        _get_tpod_address_from_param(ip_param);
        _connect();

        // Setup subscriptions and advertisements
        _steering_sub = _ros_nh.subscribe("cmd_steering_angle", 1, &TPodDriver::steering_wheel_sub_cb, this);
        _accelerator_sub = _ros_nh.subscribe("cmd_accelerator_position", 1, &TPodDriver::accelerator_sub_cb, this);
        _brake_sub = _ros_nh.subscribe("set_brake_position", 1, &TPodDriver::brake_sub_cb, this);

        _ticks_left_pub = _ros_nh.advertise<std_msgs::Int64>("ticks_left", 100);
        _ticks_right_pub = _ros_nh.advertise<std_msgs::Int64>("ticks_right", 100);
        _brake_pos_pub = _ros_nh.advertise<std_msgs::Float64>("brake_position", 100);
        _accelerator_pos_pub = _ros_nh.advertise<std_msgs::Float64>("accelerator_position", 100);
        _steering_wheel_pos_pub = _ros_nh.advertise<std_msgs::Int64>("steering_wheel_position", 100);

        _publishing_timer = _ros_nh.createTimer(ros::Duration(0.1), &TPodDriver::_publish_states, this);
        _send_setpoints_timer = _ros_nh.createTimer(ros::Duration(0.1), &TPodDriver::_send_setpoints, this);

    }

    ~TPodDriver(){
        _socket.cancel();
        _socket.close();
        _io_service.stop();
        _io_service_thread.join();
    }

    /// Steering Angle Subscription Callback
    void steering_wheel_sub_cb(const std_msgs::Float64::ConstPtr& new_setpoint)
    {
        std::lock_guard<std::mutex> lg{_setpoint_mtx};
        _steering_wheel_pos_setpoint = new_setpoint->data;
    }

    /// Accelerator Subscription Callback
    void accelerator_sub_cb(const std_msgs::Float64::ConstPtr& new_setpoint)
    {
        std::lock_guard<std::mutex> lg{_setpoint_mtx};
        _accelerator_pos_setpoint = new_setpoint->data;
    }

    /// Brake Subscription Callback
    void brake_sub_cb(const std_msgs::Float64::ConstPtr& new_setpoint)
    {
        std::lock_guard<std::mutex> lg{_setpoint_mtx};
        _brake_pos_setpoint = new_setpoint->data;
    }


private:
    // Connection stuff
    boost::asio::io_service _io_service;
    std::thread _io_service_thread{};
    boost::asio::ip::udp::socket _socket{_io_service, boost::asio::ip::udp::v4()};
    boost::asio::ip::udp::endpoint _tpod_ep;
    boost::asio::ip::udp::endpoint _remote_ep;
    std::array<char, 512> _recv_array;

    // States
    // Ticks from encoders on the rear wheels
    int _ticks_left{0}, _ticks_right{0};
    // Angular velocity of the wheels (reported from the T.Pod)
    double _angular_vel_left{0.0}, _angular_vel_right{0.0};
    // Linear and angular velocities of the T.Pod (computed by this object)
    double _linear_vel_x{0.0}, _angular_vel_z{0.0};
    // Brake and accelerator positions
    double _brake_pos{0.0}, _accelerator_pos{0.0};
    // Steering Wheel
    double _steering_wheel_pos{0.0}, _steering_wheel_vel{0.0};
    // Autonomy active
    bool _autonom;
    // Emergency stop activated
    bool _emergency_stop;

    // Setpoints
    double _accelerator_pos_setpoint{0.0};
    double _steering_wheel_pos_setpoint{0.0};
    double _steering_wheel_vel_setpoint{0.0};
    double _brake_pos_setpoint{2.4};

    // Mutexes to avoid data races
    std::mutex _state_mtx;
    std::mutex _setpoint_mtx;

    // ROS subscriptions and advertisements and stuff
    ros::NodeHandle _ros_nh;

    ros::Subscriber _steering_sub,
                    _accelerator_sub,
                    _brake_sub;

    ros::Publisher _ticks_left_pub,
                   _ticks_right_pub,
                   _angular_vel_left_pub,
                   _angular_vel_right_pub,
                   _linear_vel_x_pub,
                   _angular_vel_z_pub,
                   _brake_pos_pub,
                   _accelerator_pos_pub,
                   _steering_wheel_pos_pub,
                   _steering_wheel_vel_pub;

    ros::Timer _publishing_timer;
    ros::Timer _send_setpoints_timer;

    /// Get the ip address and port from the ROS parameter server
    ///
    /// @param[in]  param_name The name of the parameter on the ROS parameter server
    ///                        that holds the address string.
    void _get_tpod_address_from_param(std::string param_name)
    {
        std::string input_ip;
        std::string ip;
        unsigned short port;

        if(! _ros_nh.getParam(param_name, input_ip))
        {
            throw(std::runtime_error("No ip address for the T.Pod specified."));
        }
        boost::regex expression("((?:\\d{1,3}\\.){3}\\d{1,3}):(\\d{5})");
        boost::cmatch what;
        if(boost::regex_match(input_ip.c_str(), what, expression))
        {
            // what[0] contains the whole string
            // what[1] contains the ip address
            // what[2] contains the port
            ip = what[1].str();
            port = stoi(what[2].str());
            ROS_INFO_STREAM("User input ip adress for the T.Pod at " << ip << ", port " << port);
        }
        else
        {
            throw(std::runtime_error("Ip address for T.Pod malformed: " + input_ip));
        }

        _tpod_ep.address(boost::asio::ip::address::from_string(ip));
        _tpod_ep.port(port);
    }

    /// Connect to the T.Pod
    ///
    /// Opens a UDP socket to the T.Pod, automatically retries if no connection is made
    /// and throws an exception if the user aborts.
    /// Actually the retry is a leftover from test code that used TCP. UDP does not
    /// do handshakes, so probably it just connects immediately even though no server is
    /// present.
    void _connect()
    {
        ROS_INFO("Connecting to T.Pod...");
        while(ros::ok())
        {
            try {
                // Binding to port 0 selects an ephemeral port
                boost::asio::ip::udp::endpoint local_ep{boost::asio::ip::udp::v4(), 0};
                _socket.bind(local_ep);
                ROS_INFO_STREAM("T.Pod driver bound to local port: " << _socket.local_endpoint().port());
                //_socket.connect(_remote_ep);
                break;
            }
            catch (boost::system::system_error error)
            {
                ROS_WARN("Connection failed, retrying in 5 seconds...");
                ros::Duration d(5);
                d.sleep();
            }
        }
        if (ros::isShuttingDown())
        {
            _socket.cancel();
            throw(std::runtime_error("Connection aborted, ROS Node was shut down."));
        }
        ROS_INFO("Connection to T.Pod established.");

        // Start the receive action
        _receive_from_tpod();

        // Start the I/O service thread
        _io_service_thread = std::thread{&TPodDriver::_run_io, this};
    }

    void _run_io()
    {
        _io_service.run();
    }

    void _receive_from_tpod()
    {
        _socket.async_receive_from(
            boost::asio::buffer(_recv_array, _recv_array.size()),
            _remote_ep,
            boost::bind(&TPodDriver::_receive_handler, this, _1, _2)
        );
    }

    void _receive_handler(const boost::system::error_code& error, std::size_t bytes_transferred)
    {
        std::lock_guard<std::mutex> lg(_state_mtx);
        std::string input_string{_recv_array.begin(), _recv_array.begin()+bytes_transferred};
        try {
            auto input_json = json::parse(input_string);
            if (input_json.find("PosL") != input_json.end()) {
                _ticks_left = input_json["PosL"];
            }
            if (input_json.find("PosR") != input_json.end()) {
                _ticks_right = input_json["PosR"];
            }
            if (input_json.find("VelL") != input_json.end()) {
                _angular_vel_left = input_json["VelL"];
            }
            if (input_json.find("VelR") != input_json.end()) {
                _angular_vel_right = input_json["VelR"];
            }
            if (input_json.find("Brake") != input_json.end()) {
                _brake_pos = input_json["Brake"];
            }
            if (input_json.find("Speed") != input_json.end()) {
                _accelerator_pos = input_json["Speed"];
            }
            if (input_json.find("Steer") != input_json.end()) {
                _steering_wheel_pos = input_json["Steer"];
            }
            if (input_json.find("steering_wheel_vel") != input_json.end()) {
                _steering_wheel_vel = input_json["steering_wheel_vel"];
            }
            if (input_json.find("Auto") != input_json.end()) {
                _autonom = input_json["Auto"];
            }
            if (input_json.find("Emer") != input_json.end()) {
                _emergency_stop = input_json["Emer"];
            }

        }
        catch (std::invalid_argument e)
        {
            ROS_WARN_STREAM("Invalid JSON received from T.Pod: " << input_string);
        }

        // Start next receive action
        _receive_from_tpod();
    }

    void _publish_states(const ros::TimerEvent& te)
    {
        std_msgs::Int64 tl;
        tl.data = _ticks_left;
        _ticks_left_pub.publish(tl);

        std_msgs::Int64 tr;
        tr.data = _ticks_right;
        _ticks_right_pub.publish(tr);

        std_msgs::Float64 bp;
        bp.data = _brake_pos;
        _brake_pos_pub.publish(bp);

        std_msgs::Float64 ap;
        ap.data = _accelerator_pos;
        _accelerator_pos_pub.publish(ap);

        std_msgs::Int64 swp;
        swp.data = _steering_wheel_pos;
        _steering_wheel_pos_pub.publish(swp);

    }

    /// Send setpoints to T.Pod
    void _send_setpoints(const ros::TimerEvent& te)
    {
        std::lock_guard<std::mutex> lg{_setpoint_mtx};

        json j;
        j["steering_angle"] = _steering_wheel_pos_setpoint;
        j["accelerator"] = _accelerator_pos_setpoint;
        j["set_brake"] = _brake_pos_setpoint;
        std::string send_string = j.dump();

        _socket.send_to(boost::asio::buffer(send_string, send_string.size()), _tpod_ep);
    }
};


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "tpod_driver_node");

    try
    {
        // Initialize driver
        TPodDriver tpod_driver("tpod_ip");
        ROS_INFO("Was the steering wheel in zero-position at startup?");

        // Now to the business
        ros::spin();
    }
    catch (std::exception& e)
    {
        std::cerr << "Unhandled exception: " << e.what() << std::endl;
        ros::shutdown();
    }

    return 0;
}
