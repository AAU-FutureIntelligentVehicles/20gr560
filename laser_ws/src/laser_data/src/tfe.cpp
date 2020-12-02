#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

// Credit: TF transform template used from group 17gr660!!!!

int main(int argc, char** argv){
    ros::init(argc, argv, "transform_publisher");
    ros::NodeHandle n;

    ros::Rate r(10.0);
    ROS_INFO("Transformations: ready");

    tf::TransformBroadcaster bc;

    while(ros::ok()) {

        tf::Transform transform1;
        transform1.setOrigin( tf::Vector3(0.51, 0, 0.06) );
        tf::Quaternion q1;
        q1.setRPY(0, 0, 0);
        transform1.setRotation(q1);
        bc.sendTransform(tf::StampedTransform(transform1, ros::Time::now(), "rear_axle", "base_link"));

	/*tf::Transform transform2;
        transform2.setOrigin( tf::Vector3(2, 2, 0) );  //(-1.02, 0, 0.27)
        tf::Quaternion q2;
        q2.setRPY(0, 0, 0); //(0, 0, 3.1416)
        transform2.setRotation(q2);
        bc.sendTransform(tf::StampedTransform(transform2, ros::Time::now(), "base_footprint", "base_link"));*/

        tf::Transform transform3;
        transform3.setOrigin( tf::Vector3(1.18, 0, 0.27) ); //(1.18 x original, 0.58 test)
        tf::Quaternion q3;
        q3.setRPY(0, 0, 0);
        transform3.setRotation(q3);
        bc.sendTransform(tf::StampedTransform(transform3, ros::Time::now(), "base_link", "laser_front"));

        tf::Transform transform4;
        transform4.setOrigin( tf::Vector3(1.07, 0, 1.38) ); //(1.07 x original, 0.57 test)
        tf::Quaternion q4;
        q4.setRPY(3.141593, 0.31, 0);
        transform4.setRotation(q4);
        bc.sendTransform(tf::StampedTransform(transform4, ros::Time::now(), "base_link", "laser_mount_link"));

        tf::Transform transform5;
        transform5.setOrigin( tf::Vector3(-0.61, 0, 0.27) );  //(-1.02 x original, -0.52 test)
        tf::Quaternion q5;
        q5.setRPY(0, 0, 3.1416); //(0, 0, 3.1416)
        transform5.setRotation(q5);
        bc.sendTransform(tf::StampedTransform(transform5, ros::Time::now(), "base_link", "laser_rear"));

        r.sleep();
    }
}
