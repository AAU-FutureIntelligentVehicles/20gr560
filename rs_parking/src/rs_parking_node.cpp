#include <iostream>
#include <string>
#include <vector>
#include <chrono> // For timing function calls

// Intel Realsense headers
#include <librealsense2/rs.hpp>

// OpenCV headers
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>

// ROS headers
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
//#include <pcl_conversions/pcl_conversions.h>
//#include <ros/tf2.h>


using namespace std::chrono; // For timing function calls


cv::RNG rng(12345);


class ParkingSpotDetector
{
public:
    explicit ParkingSpotDetector(int frame_width=1280, int frame_height=720)
    {
        //pc_publisher = nh.advertise<sensor_msgs::PointCloud2>("/parking_spot/point_cloud", 10);
        width = frame_width;
        height = frame_height;
        frame_counter = 0;
        configure(true);
    }

    void detect()
    {
        //pipe.get_active_profile().get_device().as<rs2::playback>().pause();
        while (ros::ok()) {
            // Block program until frames arrive
            //pipe.get_active_profile().get_device().as<rs2::playback>().resume();
            frames = pipe.wait_for_frames();
            //pipe.get_active_profile().get_device().as<rs2::playback>().pause();

            // Start timing the iteration
            auto start = high_resolution_clock::now();

            // Align color to depth frame
            frames = align_to_color.process(frames);

            // Get a frame of images
            depth = frames.get_depth_frame();
            color = frames.get_color_frame();

            cv::Mat color_img(cv::Size(width, height), CV_8UC3, (void*)color.get_data(), cv::Mat::AUTO_STEP);
            cv::cvtColor(color_img, color_img, cv::COLOR_RGB2BGR);

            // Create point cloud
            auto points = rs_pointcloud.calculate(depth);
            cv::Mat xyz_img(cv::Size(width, height), CV_32FC3, (void*)points.get_data(), cv::Mat::AUTO_STEP);

            // Transformation of point cloud
            cv::transform(xyz_img, transformed_img, rotation_matrix);  // Rotation
            cv::add(transformed_img, translation_matrix, transformed_img);  // Translation

            // Extract z-values (height)
            std::vector<cv::Mat> channels;
            cv::split(transformed_img, channels);

            // Threshold for 0.05m above ground level and 8.0m to the right of the car
            cv::Mat ground_plane(cv::Size(width, height), CV_32FC1);
            cv::Mat road_shoulder(cv::Size(width, height), CV_32FC1);
            cv::threshold(channels[2], ground_plane, 0.05, 1.0, cv::THRESH_BINARY_INV);
            cv::threshold(channels[1], road_shoulder, -8.0, 1.0, cv::THRESH_BINARY);

            // Combine ground plane and road shoulder
            ground_plane.convertTo(ground_plane, CV_8U, 255, 0);
            road_shoulder.convertTo(road_shoulder, CV_8U, 255, 0);
            cv::bitwise_and(ground_plane, road_shoulder, ground_plane);

            cv::Mat closed_ground_plane;
            cv::Mat close_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(10, 10));
            cv::morphologyEx(ground_plane, closed_ground_plane, cv::MORPH_CLOSE, close_kernel);

            // Noise filtering
            cv::medianBlur(color_img, color_img, 5);
            //cv::GaussianBlur(color_img, color_img, cv::Size(5,5), 1);

            // ===========================================================
            // ================== CANNY EDGE DETECTION ===================
            // ===========================================================
            // Find edges
            cv::Mat edges;
            cv::Mat big_edges;
            cv::Canny(color_img, edges, 100, 255);

            // Filter edges
            cv::bitwise_and(edges, ground_plane, edges);
            cv::Mat dilation_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
            cv::dilate(edges, big_edges, dilation_kernel);

            // ===========================================================
            // ============== CONNECTED COMPONENT ANALYSIS ===============
            // ===========================================================
            // Find contours with CCA
            std::vector<std::vector<cv::Point> > contours;
            std::vector<cv::Vec4i> hierarchy;
            cv::findContours(big_edges, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

            cv::Mat drawing = cv::Mat::zeros(edges.size(), CV_8UC3);
            cv::Mat intersections = cv::Mat::zeros(edges.size(), CV_8UC1);
            cv::Mat markings = cv::Mat::zeros(edges.size(), CV_8UC1);
            cv::Mat lines_drawing = cv::Mat::zeros(edges.size(), CV_8UC1);
            cv::Mat parking_spot_boxes = cv::Mat::zeros(edges.size(), CV_8UC3);
            double area;
            cv::RotatedRect bounding_rect;
            std::vector<std::vector<cv::Point3f> > contours_xyz;
            std::vector<cv::Vec4f> fitted_line_uv;
            std::vector<cv::Vec6f> fitted_line_xyz;

            // Iterate over every contour
            for(size_t i = 0; i < contours.size(); i++){
                area = cv::contourArea(contours[i]);
                if (area < 1000) {
                    continue;
                }
                bounding_rect = cv::minAreaRect(contours[i]);

                // Get mostly perpendicular rectangles, set threshold t >= 5. Set t <= 1/5 for mostly parallel
                if (bounding_rect.size.height/bounding_rect.size.width < 5){ // x < 5???? WAT
                    continue;
                }

                // Allocate contour vectors
                std::vector<cv::Point3f> temp_contour_vector;
                cv::Vec4f temp_fitted_line_uv;
                cv::Vec6f temp_fitted_line_xyz;
                std::vector<cv::Point2f> rect_points_vector;

                // Fill contour vector with xyz values of all vertices of the contour rect
                for (auto & pixel : contours[i]){
                    temp_contour_vector.push_back(xyzAtPoint(pixel, transformed_img));
                    rect_points_vector.push_back(pixel);  // Gather all pixels in contour for line fitting
                }
                // Add new xyz-vertice vector to vector holding all contours
                contours_xyz.push_back(temp_contour_vector);

                cv::fitLine(rect_points_vector, temp_fitted_line_uv, cv::DIST_L2, 0, 0.01, 0.01);
                cv::fitLine(temp_contour_vector, temp_fitted_line_xyz, cv::DIST_L2, 0, 0.01, 0.01);

                // If the slope of the fitted line is close to 0 --> line is mostly perpendicular
                if (std::abs(-temp_fitted_line_xyz[0] / temp_fitted_line_xyz[1]) < 0.25){
                    // Save fitted line in a vector
                    fitted_line_uv.push_back(temp_fitted_line_uv);
                    fitted_line_xyz.push_back(temp_fitted_line_xyz);

                    cv::drawContours(markings,
                                     contours,
                                     (int) i,
                                     cv::Scalar(255),
                                     cv::FILLED,
                                     cv::LINE_8,
                                     hierarchy,
                                     0);

                    ParkingSpotDetector::drawFittedLine(lines_drawing, temp_fitted_line_uv);

                    // Find the intersections between fitted lines and edges of markings
                    cv::Mat se = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(25, 25));
                    cv::morphologyEx(markings, markings, cv::MORPH_CLOSE, se);
                    cv::bitwise_and(markings, lines_drawing, intersections);

                    //cv::cvtColor(intersections, intersections, cv::COLOR_BGR2GRAY);
                    //cv::dilate(intersections, intersections, dilation_kernel);
                    std::vector<std::vector<cv::Point> > intersection_contours;
                    std::vector<cv::Vec4i> intersection_hierarchy;
                    std::vector<cv::Point> box_points;
                    std::vector<std::vector<cv::Point> > box_vector;
                    cv::findContours(intersections, intersection_contours, intersection_hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

                    //ParkingSpotDetector::drawFittedLine(drawing, temp_fitted_line_uv);
                    for (int k = 0; k < intersection_contours.size(); ++k){
                        cv::drawContours(drawing,
                                         intersection_contours,
                                         k,
                                         cv::Scalar(127, 255, 0),
                                         1,
                                         cv::LINE_8,
                                         intersection_hierarchy,
                                         0);
                        // sort points, left to right (x axis)
                        auto val = std::minmax_element(intersection_contours[k].begin(),
                                                       intersection_contours[k].end(),
                                                       [](cv::Point const& a, cv::Point const& b){
                                                           return a.x < b.x;
                                                       });

                        if (k == 0) {
                            box_points.push_back(*val.first);
                            box_points.push_back(*val.second);
                            continue;
                        } else {
                            box_points.push_back(*val.second);
                            box_points.push_back(*val.first);
                        }
                        box_vector.push_back(box_points);
                        box_points.clear();
                        box_points.push_back(*val.first);
                        box_points.push_back(*val.second);

                        // ===========================================================
                        // ==================== CLASSIFICATION =======================
                        // ===========================================================
                        // Width: distance between points parallel to driving direction
                        // Length: distance between points perpendicular to driving direction
                        float spot_width = std::abs(xyAtPoint(box_points[1], transformed_img).x - xyAtPoint(box_points[3], transformed_img).x);
                        float spot_length = std::abs(xyAtPoint(box_points[0], transformed_img).y - xyAtPoint(box_points[1], transformed_img).y);
                        // Does this even make sense..?
                        std::cout << spot_width << " " << spot_length << std::endl;
                        if ((spot_width < 1.2 && spot_length < 2.4) || (spot_width < 2.4 && spot_length < 1.2)) {
                            continue;
                        }
                        cv::Point2f parking_spot_center;
                        if (isSpotVacant(box_vector[k-1], ground_plane, parking_spot_center)){
                            cv::drawContours(color_img,
                                             box_vector,
                                             k-1,
                                             cv::Scalar(0, 200, 0),
                                             3,
                                             cv::LINE_8,
                                             hierarchy,
                                             0);
                        } else {
                            cv::drawContours(color_img,
                                             box_vector,
                                             k-1,
                                             cv::Scalar(0, 0, 200),
                                             3,
                                             cv::LINE_8,
                                             hierarchy,
                                             0);
                        }
                        // TODO: publish point

                        /*
                        // compare y axis
                        val = std::minmax_element(intersection_contours[k].begin(), intersection_contours[k].end(), [](cv::Point const& a, cv::Point const& b){
                            return a.y < b.y;
                        });

                        std::cout << " TopMost [ " << val.first->x << ", " << val.first->y << " ]" << std::endl;
                        std::cout << " BottomMost [ " << val.second->x << ", " << val.second->y << " ]" << std::endl;
                         */
                    }
                }
            }

            // Stop timing
            auto stop = high_resolution_clock::now();
            auto duration = duration_cast<milliseconds>(stop-start);
            std::cout << "Loop iteration time: " << duration.count() << "ms" << std::endl;
            /*
            // ===========================================================
            // ============= STANDARD LINE HOUGH TRANSFORM ===============
            // ===========================================================
            cv::Mat hough_lines_mat = cv::Mat::zeros(edges.size(), CV_8UC3);
            std::vector<cv::Vec2f> lines;
            HoughLines(edges, lines, 1, CV_PI/180, 150, 0, 0);
            // Draw the lines
            for (auto & line : lines)
            {
                ParkingSpotDetector::drawHoughLine(line, hough_lines_mat);
            }

            // ===========================================================
            // ======== PROGRESSIVE PROBABILISTIC HOUGH TRANSFORM ========
            // ===========================================================
            // Detect lines given edge features (Probabilistic Hough Transform)
            std::vector<cv::Vec4i> hough_lines;
            std::vector<cv::Vec4i> perpendicular_hough_lines;
            cv::Mat boxes(cv::Size(width, height), CV_8UC1, cv::Scalar(0));
            HoughLinesP(edges, hough_lines, 1, CV_PI / 180, 100, 150, 30);

            // Sort the lines, closest lines first
            std::sort(hough_lines.begin(),
                      hough_lines.end(),
                      [&, this](const cv::Vec4i &line1, const cv::Vec4i &line2) {
                        // TODO: Make this readable
                        double line1_distance = std::min(cv::norm(xyAtPoint(cv::Point(line1[0], line1[1]), transformed_img)),
                                                         cv::norm(xyAtPoint(cv::Point(line1[2], line1[3]), transformed_img)));
                        double line2_distance = std::min(cv::norm(xyAtPoint(cv::Point(line2[0], line2[1]), transformed_img)),
                                                         cv::norm(xyAtPoint(cv::Point(line2[2], line2[3]), transformed_img)));

                        return (line1_distance < line2_distance);  // Evaluates to either true or false
                      });

            for(auto & hough_line : hough_lines){
                cv::Point pt1(hough_line[0], hough_line[1]), pt2(hough_line[2], hough_line[3]);
                cv::Point3f pt1_xyz, pt2_xyz;
                pt1_xyz = xyzAtPoint(pt1, transformed_img);
                pt2_xyz = xyzAtPoint(pt2, transformed_img);
                // If hough_line is not approximately perpendicular to driving direction: skip it
                if (std::abs((pt2_xyz.x-pt1_xyz.x)/(pt2_xyz.y-pt1_xyz.y)) > 0.25) {
                    continue;
                }
                // Else, keep it
                perpendicular_hough_lines.push_back(hough_line);
                //cv::line(color_img, pt1, pt2, cv::Scalar(255, 0, 0), 2, cv::LINE_8);
            }
            // TODO: Merge line segments from Probabilistic Hough Transform

            if (perpendicular_hough_lines.size() > 1) {
                for (int i = 0; i < perpendicular_hough_lines.size()-1; ++i){
                    // Current line in vector
                    cv::Point line1_pt1(perpendicular_hough_lines[i][0], perpendicular_hough_lines[i][1]);
                    cv::Point line1_pt2(perpendicular_hough_lines[i][2], perpendicular_hough_lines[i][3]);
                    cv::Point3f line1_pt1_xyz, line1_pt2_xyz;
                    line1_pt1_xyz = xyzAtPoint(line1_pt1, transformed_img);
                    line1_pt2_xyz = xyzAtPoint(line1_pt2, transformed_img);

                    // Next line in vector
                    cv::Point line2_pt1(perpendicular_hough_lines[i+1][0], perpendicular_hough_lines[i+1][1]);
                    cv::Point line2_pt2(perpendicular_hough_lines[i+1][2], perpendicular_hough_lines[i+1][3]);
                    cv::Point3f line2_pt1_xyz, line2_pt2_xyz;
                    line2_pt1_xyz = xyzAtPoint(line2_pt1, transformed_img);
                    line2_pt2_xyz = xyzAtPoint(line2_pt2, transformed_img);

                    if (std::abs(line1_pt1_xyz.x - line2_pt1_xyz.x) < 0.5 or
                        std::abs(line1_pt2_xyz.x - line2_pt2_xyz.x) < 0.5 or
                        std::abs(line1_pt1_xyz.x - line2_pt2_xyz.x) < 0.5)

                    {
                        continue;
                    }

                    cv::Point2f vertices[4];
                    vertices[0] = cv::Point2f(line1_pt1.x, line1_pt1.y);
                    vertices[1] = cv::Point2f(line1_pt2.x, line1_pt2.y);
                    vertices[2] = cv::Point2f(line2_pt2.x, line2_pt2.y);
                    vertices[3] = cv::Point2f(line2_pt1.x, line2_pt1.y);
                    float mean_x = 0;
                    float mean_y = 0;
                    for (int j = 0; j < 4; ++j){
                        mean_x = mean_x + vertices[j].x;
                        mean_y = mean_y + vertices[j].y;
                        //cv::line(boxes, vertices[j], vertices[(j+1)%4], cv::Scalar(255), 1);
                        cv::line(color_img, vertices[j], vertices[(j+1)%4], cv::Scalar(0, 200, 0), 2);
                    }
                    mean_x = mean_x/4;
                    mean_y = mean_y/4;
                    // TODO: Publish point
                }
            }

            // ===========================================================
            // ================== FAST CORNER DETECTION ==================
            // ===========================================================
            // Detect corners using the FAST method
            std::vector<cv::KeyPoint> corners;
            cv::Mat grayscale;
            cv::cvtColor(color_img, grayscale, cv::COLOR_BGR2GRAY);
            cv::FAST(grayscale, corners, 50, true, cv::FastFeatureDetector::TYPE_9_16);
            cv::KeyPointsFilter::runByPixelsMask(corners, ground_plane);

            // TODO: Group corners, fit line segment to each group

            // ================== RANSAC ==================
            // TODO
            */
            // Visualize results
            cv::cvtColor(ground_plane, ground_plane, cv::COLOR_GRAY2BGR);
            cv::Mat masked_color;
            cv::bitwise_and(color_img, ground_plane, masked_color);
            cv::putText(color_img, //target image
                        "frame: " + std::to_string(++frame_counter), //text
                        cv::Point(10, 30), //top-left position
                        cv::FONT_HERSHEY_SIMPLEX,
                        0.5,
                        CV_RGB(80, 0, 200), //font color
                        1);
            //cv::drawKeypoints(masked_color, corners, masked_color, cv::Scalar(0,0,255));
            //cv::imwrite("/home/rns/Documents/Robotics/semester5/p5/tests/perp/" + std::to_string(frame_counter) + ".png", color_img);
            //cv::imshow(xyz_window, hough_lines_mat);
            cv::imshow(transformed_window, edges);
            cv::imshow(ground_window, masked_color);
            cv::imshow(rgb_window, color_img);
            cv::waitKey(5);


            /*
            sensor_msgs::PointCloud2 pc_msg;
            pcl::toROSMsg(*pcl_cloud, pc_msg);
            pc_msg.header.frame_id = "camera_optical_depth_frame";
            pc_publisher.publish(pc_msg);
            */
        }

        pipe.stop();
    }

private:
    rs2::pipeline realtimePipeSetup()
    {
        cfg.enable_stream(RS2_STREAM_DEPTH, 1280, 720, RS2_FORMAT_Z16, 30);
        cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_BGR8, 30);
        pipe.start(cfg);
        return pipe;
    }

    rs2::pipeline playbackPipeSetup()
    {
        // Files: 20201204_101527, 20201204_131943
        // others: 20201204_084450, 20201204_084636
        cfg.enable_device_from_file("/home/rns/Documents/Robotics/semester5/p5/workspaces/melodic/20201204_084636.bag");
        pipe.start(cfg);
        return pipe;
    }

    void configure(bool play_from_recording)
    {
        bool playback = play_from_recording;
        // Create a Pipeline - this serves as a top-level API for streaming and processing frames
        // Also configures and starts pipeline
        pipe = playbackPipeSetup();
        if (!playback) {
            pipe = realtimePipeSetup();
        }

        frames = pipe.wait_for_frames();
        cv::namedWindow(rgb_window);
        cv::namedWindow(xyz_window);
        cv::namedWindow(transformed_window);
        cv::namedWindow(ground_window);
        //rs2::align align_to_depth(RS2_STREAM_DEPTH);  // Possibility to align color image to depth image
        align_to_color = rs2::align(RS2_STREAM_COLOR);
        depth = frames.get_depth_frame();
        color = frames.get_color_frame();
        width = color.get_width();
        height = color.get_height();

        // Original rotation matrix
        /*cv::Matx33f rotation_matrix(-0.7071, 0.3536, -0.6124,
                                    0.7071, 0.3536, -0.6124,
                                    0.0000, -0.8660, -0.5000);*/
        rotation_matrix = cv::Matx33f(-0.7008, 0.3658, -0.6124,
                                      0.7132, 0.3412, -0.6124,
                                      -0.0151 , -0.8659, -0.5000);
        translation_vector = cv::Scalar(-0.02,
                                        -0.41,
                                        1.63);
        translation_matrix = cv::Mat(cv::Size(1280, 720), CV_32FC3, translation_vector);

        // Wait for auto exposure to settle before starting detection
        for (int i = 0; i < 30; ++i) {
            frames = pipe.wait_for_frames();
            depth = frames.get_depth_frame();
            color = frames.get_color_frame();
        }

        // Proper memory allocation for matrices used during detection
        transformed_img = cv::Mat(cv::Size(width, height), CV_32FC3);
    }

    static cv::Point3f xyzAtPoint(const cv::Point &src, const cv::Mat &xyz_image)
    {
        return xyz_image.at<cv::Point3f>(src);
    }

    static cv::Point2f xyAtPoint(const cv::Point &src, const cv::Mat &xyz_image)
    {
        auto xyz = xyz_image.at<cv::Point3f>(src);
        return {xyz.x, xyz.y};
    }

    static void drawHoughLine(const cv::Vec2f &hough_line, const cv::Mat &dst)
    {
        float rho = hough_line[0], theta = hough_line[1];
        cv::Point pt1, pt2;
        double a = cos(theta), b = sin(theta);
        double x0 = a*rho, y0 = b*rho;
        pt1.x = cvRound(x0 + 1000*(-b));
        pt1.y = cvRound(y0 + 1000*(a));
        pt2.x = cvRound(x0 - 1000*(-b));
        pt2.y = cvRound(y0 - 1000*(a));
        cv::line(dst, pt1, pt2, cv::Scalar(0,0,255), 1, cv::LINE_8);
    }

    static void drawFittedLine(const cv::Mat &img, const cv::Vec4f &fitted_line)
    {
        // vx = fitted_line[0]; vy = fitted_line[1]; x0 = fitted_line[2]; y0 = fitted_line[3]
        cv::Point point0(fitted_line[2], fitted_line[3]);
        double k = fitted_line[1] / fitted_line[0];

        cv::Point point1(0, k * (0 - point0.x) + point0.y);
        cv::Point point2(img.cols, k * (img.cols - point0.x) + point0.y);

        cv::line(img, point1, point2, cv::Scalar(255), 2, cv::LINE_8, 0);
    }

    bool isLineCloser(const cv::Vec4i &line1, const cv::Vec4i &line2)
    {
        double line1_distance = std::min(norm(xyAtPoint(cv::Point(line1[0], line1[1]), transformed_img)),
                                      norm(xyAtPoint(cv::Point(line1[2], line1[3]), transformed_img)));
        double line2_distance = std::min(norm(xyAtPoint(cv::Point(line2[0], line2[1]), transformed_img)),
                                         norm(xyAtPoint(cv::Point(line2[2], line2[3]), transformed_img)));

        return (line1_distance < line2_distance);  // Evaluates to either true or false
    }

    bool isSpotVacant(const std::vector<cv::Point> &spot_contour, const cv::Mat &road_plane, cv::Point2f &spot_center){
        cv:: Mat contour_on_road(cv::Size(road_plane.size()), CV_8UC1);
        std::vector<std::vector<cv::Point> > fake_contours;
        fake_contours.push_back(spot_contour);
        cv::drawContours(contour_on_road, fake_contours, 0, cv::Scalar(255), cv::FILLED);
        cv::Moments mu = moments(spot_contour);
        double spot_area = cv::countNonZero(contour_on_road);
        cv::bitwise_and(contour_on_road, road_plane, contour_on_road);
        double spot_area_on_road = cv::countNonZero(contour_on_road);
        double on_road_ratio = spot_area_on_road / spot_area;


        if (on_road_ratio > 0.93) {
            //add 1e-5 to avoid division by zero
            spot_center = cv::Point2f(static_cast<float>(mu.m10 / (mu.m00 + 1e-5)),
                             static_cast<float>(mu.m01 / (mu.m00 + 1e-5)));
            return true;
        }
        return false;
    }

    // Allocate variables
    //ros::NodeHandle nh;
    //ros::Publisher pc_publisher;
    rs2::pipeline pipe;
    rs2::config cfg;
    rs2::pointcloud rs_pointcloud;
    rs2::frameset frames;
    rs2::depth_frame depth = rs2::depth_frame(rs2::frame());
    rs2::video_frame color = rs2::video_frame(rs2::frame());
    rs2::align align_to_color = rs2::align(RS2_STREAM_COLOR);
    int width;
    int height;
    int frame_counter;
    std::string rgb_window = "D455 RGB Image";
    std::string xyz_window = "D455 XYZ Image";
    std::string transformed_window = "trans";
    std::string ground_window = "ground";

    cv::Matx33f rotation_matrix;
    cv::Scalar translation_vector;
    cv::Mat translation_matrix;

    cv::Mat transformed_img;

};

int main(int argc, char *argv[])
{
	std::cout << "Compiled using OpenCV version " << CV_VERSION << std::endl;
    ros::init(argc, argv, "rs_parking_detector");
    ParkingSpotDetector detector;
    detector.detect();
	ros::shutdown();
	return 0;
}

/* OPTIONAL: Post-processing -> (Decimation, hole-filling?)
 * DONE Transformation -> (Rotation matrix, plus translation to a transformed image)
 * DONE Road surface extraction -> (align xyz indices with rgb image, thresholding, consider closing afterwards)
 * DONE Marking extraction -> (FAST feature detector vs thresholding vs edge detection)
 * DONE Virtual line determination -> (RANSAC vs just assume that the corners (of the blobs) are good)
 * DONE Spot corner determination -> (Intersection between lines)
 * Perpendicular vs parallel determination -> (Distance between corners)
 * DONE Classification of vacancy -> (K-means or naïve Bayes or just a threshold of a parameter)
 * Navigation goal estimation (Halfway distance between the corners, orientation can be assumed based on type of spot)
 */

/*
 * ====================================
 * REMOVED OLD CODE
 * Kept because it hurts to let go :'(
 * ====================================
 */

/*
#include <tuple>
#include <thread>

// Boost
#include <boost/thread/thread.hpp>

// PCL headers
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <pcl/console/parse.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>


void Preprocess(cv::Mat img)
{
    // cv::Mat img = image_ptr->image;

    // Convert color space and split color channels
    //cv::cvtColor(image_ptr->image, image_ptr->image, cv::COLOR_BGR2GRAY);

    cv::cvtColor(img, img, cv::COLOR_BGR2HSV);
    std::vector<cv::Mat> channels;
    cv::split(img, channels);

    //cv::equalizeHist(channels[0], channels[0]);
    cv::normalize(channels[2], channels[2], 0, 255, cv::NORM_MINMAX);
    //cv::equalizeHist(channels[1], channels[1]);

    // Reassemble color channels of hist-stretched image
    cv::merge(channels, img);
}

void ExtractFeatures(cv::Mat img)
{
    // * Thresholds:
    // * H: 0-75
    // * S: 0-40
    // * V: 105-215
    //
    cv::Scalar means = cv::mean(img);
    cv::inRange(img, cv::Scalar(0, 1, 105), cv::Scalar(70, means[1]*0.77, 215), img);
    cv::Mat dilate_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3), cv::Point(1, 1));
    cv::dilate(img, img, dilate_kernel);

    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(img, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    cv::Mat drawing = cv::Mat::zeros( img.size(), CV_8UC3 );
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
                cv::Scalar color = cv::Scalar(rng.uniform(0, 256),
                                              rng.uniform(0,256),
                                              rng.uniform(0,256));
                cv::drawContours(drawing, contours, (int)i, color, 2, cv::LINE_8, hierarchy, 0);
            }
        }
    }
    imshow("Contours", drawing);
}

static void onMouse(int event, int x, int y, int flags, void* param) // now it's in param
{
    cv::Mat &xyz = *((cv::Mat*)param); //cast and deref the param

    if (event == cv::EVENT_LBUTTONDOWN)
    {
        cv::Vec3f val = xyz.at<cv::Vec3f>(y,x); // opencv is row-major !
        std::cout << "u= " << x << " v= " << y << "val= "<<val<< std::endl;
    }
}

std::tuple<int, int, int> rgb_texture(rs2::video_frame texture, rs2::texture_coordinate texture_xy)
{
    // Get Width and Height coordinates of texture
    int width  = texture.get_width();  // Frame width in pixels
    int height = texture.get_height(); // Frame height in pixels

    // Normals to Texture Coordinates conversion
    int x_value = std::min(std::max(int(texture_xy.u * width  + .5f), 0), width - 1);
    int y_value = std::min(std::max(int(texture_xy.v * height + .5f), 0), height - 1);

    int bytes = x_value * texture.get_bytes_per_pixel();   // Get # of bytes per pixel
    int strides = y_value * texture.get_stride_in_bytes(); // Get line width in bytes
    int text_index =  (bytes + strides);

    const auto new_texture = reinterpret_cast<const uint8_t*>(texture.get_data());

    // RGB components to save in tuple
    int NT1 = new_texture[text_index];
    int NT2 = new_texture[text_index + 1];
    int NT3 = new_texture[text_index + 2];

    return std::tuple<int, int, int>(NT1, NT2, NT3);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_conversion_rgb(const rs2::points& points, const rs2::video_frame& color){

    // Object Declaration (Point Cloud)
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    // Declare Tuple for RGB value Storage (<t0>, <t1>, <t2>)
    std::tuple<uint8_t, uint8_t, uint8_t> rgb_color;

    // Convert data captured from Realsense camera to Point Cloud
    auto sp = points.get_profile().as<rs2::video_stream_profile>();

    cloud->width  = static_cast<uint32_t>( sp.width()  );
    cloud->height = static_cast<uint32_t>( sp.height() );
    cloud->is_dense = false;
    cloud->points.resize( points.size() );

    auto Texture_Coord = points.get_texture_coordinates();
    auto Vertex = points.get_vertices();

    // Iterating through all points and setting XYZ coordinates and RGB values
    for (int i = 0; i < points.size(); i++)
    {
        // Mapping Depth Coordinates. Depth data stored as XYZ values
        cloud->points[i].x = Vertex[i].x;
        cloud->points[i].y = Vertex[i].y;
        cloud->points[i].z = Vertex[i].z;

        // Obtain color texture for specific point
        rgb_color = rgb_texture(color, Texture_Coord[i]);

        // Mapping Color (BGR due to Camera Model)
        cloud->points[i].r = std::get<2>(rgb_color); // Reference tuple<2>
        cloud->points[i].g = std::get<1>(rgb_color); // Reference tuple<1>
        cloud->points[i].b = std::get<0>(rgb_color); // Reference tuple<0>

    }

	return cloud; // PCL RGB Point Cloud generated
}

pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_conversion(const rs2::points& points, const rs2::video_frame& image_frame){

    // Object Declaration (Point Cloud)
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // Convert data captured from Realsense camera to Point Cloud
    auto sp = points.get_profile().as<rs2::video_stream_profile>();

    cloud->width  = static_cast<uint32_t>( sp.width()  );
    cloud->height = static_cast<uint32_t>( sp.height() );
    cloud->is_dense = false;
    cloud->points.resize( points.size() );

    auto Vertex = points.get_vertices();

    // Iterating through all points and setting XYZ coordinates and RGB values
    for (int i = 0; i < points.size(); i++)
    {
        // Mapping Depth Coordinates. Depth data stored as XYZ values
        cloud->points[i].x = Vertex[i].x;
        cloud->points[i].y = Vertex[i].y;
        cloud->points[i].z = Vertex[i].z;
    }

   return cloud; // PCL RGB Point Cloud generated
}

pcl::visualization::PCLVisualizer::Ptr simpleVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  //viewer->addCoordinateSystem (1.0, "global");
  viewer->initCameraParameters ();
  return (viewer);
}


void sac_plane_segmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	//pcl::io::savePCDFileASCII("pc.pcd", *cloud);
	pcl::PointCloud<pcl::PointXYZ>::Ptr vis_cloud (new pcl::PointCloud<pcl::PointXYZ>);

	// Outlier removal
	//pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	//sor.setInputCloud(cloud);
	//sor.setMeanK(5);
	//sor.setStddevMulThresh(1.0);
	//sor.filter(*cloud_filtered);

	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	// Optional
	seg.setOptimizeCoefficients(true);
	// Mandatory
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(1000);
	seg.setDistanceThreshold(0.05);
	//seg.setNumberOfThreads(8);

	seg.setInputCloud(cloud);
	seg.segment(*inliers, *coefficients);

	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*cloud);
    std::cout << inliers->indices.size() << std::endl;

    //pcl::io::savePCDFileASCII("pc_filt.pcd", *cloud_filtered);
    //pcl::io::savePCDFileASCII("plane_extract.pcd", *vis_cloud);

}


void region_grower(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	pcl::search::Search<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
	//pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normal_estimator;
	pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
	//normal_estimator.setSearchMethod(tree);
	normal_estimator.setNormalEstimationMethod (normal_estimator.AVERAGE_3D_GRADIENT);
    normal_estimator.setMaxDepthChangeFactor(0.02f);
    normal_estimator.setNormalSmoothingSize(10.0f);
	normal_estimator.setInputCloud(cloud);
	//normal_estimator.setKSearch(300);
	std::cout << "Computing normals..." << std::endl;
	normal_estimator.compute(*normals);

	pcl::IndicesPtr indices(new std::vector <int>);
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(cloud);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(0.0, 1.0);
	std::cout << "Filtering..." << std::endl;
	pass.filter(*indices);

	pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
	reg.setMinClusterSize(5000);
	reg.setMaxClusterSize(1000000);
	reg.setSearchMethod(tree);
	reg.setNumberOfNeighbours(30);
	reg.setInputCloud(cloud);
	  //reg.setIndices (indices);
	reg.setInputNormals(normals);
	reg.setSmoothnessThreshold(3.0 / 180.0 * M_PI);
	reg.setCurvatureThreshold(1.0);

	std::vector <pcl::PointIndices> clusters;
	std::cout << "Region growing commencing..." << std::endl;
	reg.extract (clusters);

	std::cout << "Number of clusters is equal to " << clusters.size() << std::endl;
	std::cout << "First cluster has " << clusters[0].indices.size() << " points." << std::endl;
	std::cout << "These are the indices of the points of the initial" << std::endl << "cloud that belong to the first cluster:" << std::endl;
	int counter = 0;
	while (counter < clusters[0].indices.size())
	{
		std::cout << clusters[0].indices[counter] << ", ";
		counter++;
		if (counter % 10 == 0) {
			std::cout << std::endl;
		}
	}
	std::cout << std::endl;

	pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();
	pcl::visualization::CloudViewer viewer("Cluster viewer");
	viewer.showCloud(colored_cloud);
	while (!viewer.wasStopped ())
	{
	}
}


pcl::IndicesPtr z_passthrough(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	pcl::IndicesPtr filtered_indices (new std::vector<int>);  // Create pointer to index vector
	pcl::PassThrough<pcl::PointXYZ> cloud_filter;             // Create the filtering object
    cloud_filter.setInputCloud(cloud);                        // Input generated cloud to filter
    cloud_filter.setFilterFieldName("z");                     // Set field name to Z-coordinate
    cloud_filter.setFilterLimits(-1.0, 1.0);                  // Set accepted interval values
    cloud_filter.filter(*filtered_indices);                   // Indices of remaining data outputted

    return filtered_indices;
}
*/

/*void z_thresholding(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, cv::Mat &output_mat, float frame_height, float frame_width)
{
	auto new_var = reinterpret_cast<void*>(cloud);
}


void convert_indices_to_cv_mask(pcl::IndicesPtr indices, cv::Mat output_mask)
{
	for (int i = 0; i < *indices.size(); ++i)
	{
		output_mask
	}
}
*/