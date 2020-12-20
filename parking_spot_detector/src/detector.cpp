#include <ros/ros.h>
#include <memory>
#include <vector>
#include <chrono>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <pcl_ros/point_cloud.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>


using namespace std::chrono;
using namespace cv;


cv::RNG rng(12345);

void Preprocess(cv_bridge::CvImagePtr image_ptr)
{
	// TODO: Consider returning a CUDA Matrix
	// cv::Mat img = image_ptr->image;

	// Convert color space and split color channels
	//cv::cvtColor(image_ptr->image, image_ptr->image, cv::COLOR_BGR2GRAY);
	
	cv::cvtColor(image_ptr->image, image_ptr->image, cv::COLOR_BGR2HSV);
	std::vector<cv::Mat> channels;
	cv::split(image_ptr->image, channels);

	//cv::equalizeHist(channels[0], channels[0]);
	cv::normalize(channels[2], channels[2], 0, 255, cv::NORM_MINMAX);
	//cv::equalizeHist(channels[1], channels[1]);

	// Reassemble color channels of hist-stretched image 
	cv::merge(channels, image_ptr->image);
}

void ExtractFeatures(cv_bridge::CvImagePtr image_ptr)
{
	// TODO: Remove sky (ROI)?
	/* Thresholds:
	 * H: 0-75
	 * S: 0-40
	 * V: 105-215
	*/
	cv::Scalar means = cv::mean(image_ptr->image);
	cv::inRange(image_ptr->image, cv::Scalar(0, 1, 105), cv::Scalar(70, means[1]*0.77, 215), image_ptr->image);
	cv::Mat dilate_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3), cv::Point(1, 1));
	cv::dilate(image_ptr->image, image_ptr->image, dilate_kernel);

    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(image_ptr->image, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    
    cv::Mat drawing = cv::Mat::zeros( image_ptr->image.size(), CV_8UC3 );
    double area;
    cv::RotatedRect bounding_rect;
    for(size_t i = 0; i< contours.size(); i++)
    {
        area = cv::contourArea(contours[i]);
        if (area > 75) {
            //cv::Point2f rect_points[4];
            //cv::Size2f rect_size;
            bounding_rect = cv::minAreaRect(contours[i]);
            if (std::max(bounding_rect.size.height, bounding_rect.size.width)/std::min(bounding_rect.size.height, bounding_rect.size.width) >= 10) {
                cv::Scalar color = cv::Scalar(rng.uniform(0, 256), rng.uniform(0,256), rng.uniform(0,256));
                cv::drawContours(drawing, contours, (int)i, color, 2, cv::LINE_8, hierarchy, 0);
            }
        }
    }
    imshow("Contours", drawing);
}


class DetectorNode
{
public:
	DetectorNode()
	{
		rgb_image_sub_ = nh_.subscribe("/camera1/color/image_raw", 3, &DetectorNode::RgbCallback, this);
		pcl_sub_ = nh_.subscribe("/camera1/depth/color/points", 3, &DetectorNode::PclCallback, this);

		nav_goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/parking_spot/center", 10);

		opencv_window_ = "Image window";
		cv::namedWindow(opencv_window_);
		ROS_INFO("Initialised node");
	}

private:
	void RgbCallback(const sensor_msgs::Image::ConstPtr& msg)
	{
		cv_bridge::CvImagePtr cv_ptr;
		try {
			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		}
		catch (cv_bridge::Exception &e) {
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}

		auto start = high_resolution_clock::now();

		Preprocess(cv_ptr);
		ExtractFeatures(cv_ptr);

		auto stop = high_resolution_clock::now();
		auto duration = duration_cast<microseconds>(stop-start);
		std::cout << duration.count() << std::endl;

		cv::imshow(opencv_window_, cv_ptr->image);
		cv::waitKey(3);
	}

	void PclCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) const
	{
		ROS_INFO("Received PointCloud2 message");
		const std::string field_name = "x";
		sensor_msgs::PointCloud2 pc = *msg;
		for (sensor_msgs::PointCloud2Iterator<float> iter_xyz(pc, field_name); iter_xyz != iter_xyz.end(); ++iter_xyz)
		{
			std::cout << iter_xyz[0] << std::endl;
			iter_xyz = iter_xyz.end();
		}
		return;
	}

	ros::NodeHandle nh_;
	ros::Publisher nav_goal_pub_;
	ros::Subscriber rgb_image_sub_, pcl_sub_;

	std::string opencv_window_;

	sensor_msgs::Image current_image;
	sensor_msgs::PointCloud2 current_pcl;
};

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "parking_spot_detector");
	ROS_INFO("Starting Parking Spot Detector...");
	DetectorNode detector;

	ros::spin();
	return 0;
}