#include <librealsense2/rs.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>


rs2::pipeline realtime_setup()
{
    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_DEPTH, 1280, 720, RS2_FORMAT_Z16, 30);
    cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_BGR8, 30);
    pipe.start(cfg);
    return pipe;
}

rs2::pipeline playback_setup()
{
    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_device_from_file("20201203_160144.bag");
    pipe.start(cfg);
    return pipe;
}

int main()
{
    auto pipeline = playback_setup();
    if (pipeline.get_active_profile().get_device().as<rs2::recorder>()) {
        std::cout << "Recording..." << std::endl;
        while (true) {}
    } else if (pipeline.get_active_profile().get_device().as<rs2::playback>()) {
        std::cout << "Playing back..." << std::endl;
        cv::namedWindow("rgb_window");
        while (true) {
            rs2::frameset frames = pipeline.wait_for_frames();
            // Try to get a frame of a depth image
            auto depth = frames.get_depth_frame();
            auto color = frames.get_color_frame();

            // Get the depth frame's dimensions
            int width = color.get_width();
            int height = color.get_height();

            cv::Mat color_img(cv::Size(width, height), CV_8UC3, (void *) color.get_data(), cv::Mat::AUTO_STEP);
            cv::imshow("rgb_window", color_img);
            cv::waitKey(1);
        }
    }
}