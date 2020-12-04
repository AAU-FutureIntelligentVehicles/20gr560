#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Odometry.h>
#include <cmath>
#include <tf/transform_broadcaster.h>
class test{
private:
    ros::NodeHandle nh;
    ros::Subscriber sub, sub1;
    ros::Publisher pub;
    ros::Time currentTime;
    nav_msgs::Odometry odom;
    double encoder_x, encoder_y, encoder_ori_x, encoder_ori_y, encoder_ori_w, encoder_ori_z, encoder_vel_x, encoder_ang_z;
    double laser_x, laser_y, laser_ori_x, laser_ori_y, laser_ori_z, laser_ori_w, laser_vel_x, laser_ang_z;
    double x, y, ori, vel_x, vel_y, ang_z;
    double siny_cosp, cosy_cosp, laser_yaw, encoder_yaw;
    double encoder_weigth = 0.2;
    double laser_weigth = 0.8;
public:
    void subscriber(){
        sub=nh.subscribe("odom_rf2o",10,&test::callback, this);
        sub1=nh.subscribe("odom", 10, &test::callback1, this);
    }
    void publisher(){
        pub=nh.advertise<nav_msgs::Odometry>("odom_merge", 1000);
    }
    void callback(const nav_msgs::Odometry::ConstPtr& laser_data){
    //ROS_INFO("X: [%f]", laser_data->pose.pose.position.x);


    laser_x = laser_data->pose.pose.position.x;
    laser_y = laser_data->pose.pose.position.y;
    laser_ori_x = laser_data->pose.pose.orientation.x;
    laser_ori_y = laser_data->pose.pose.orientation.y;
    laser_ori_z = laser_data->pose.pose.orientation.z;
    laser_ori_w = laser_data->pose.pose.orientation.w;

    siny_cosp = 2 * (laser_ori_w * laser_ori_z + laser_ori_x * laser_ori_y);
    cosy_cosp = 1 - 2 * (laser_ori_y * laser_ori_y + laser_ori_z * laser_ori_z);
    laser_yaw = std::atan2(siny_cosp, cosy_cosp); //rotation


    laser_vel_x = laser_data->twist.twist.linear.x;
    laser_ang_z = laser_data->twist.twist.angular.z;
    //ROS_INFO("Orientation: [%f]", laser_yaw);
}
void callback1(const nav_msgs::Odometry::ConstPtr& encoder_data){
    encoder_x = encoder_data->pose.pose.position.x;
    encoder_y = encoder_data->pose.pose.position.y;
    encoder_ori_x = encoder_data->pose.pose.orientation.x;
    encoder_ori_y = encoder_data->pose.pose.orientation.y;
    encoder_ori_z = encoder_data->pose.pose.orientation.z;
    encoder_ori_w = encoder_data->pose.pose.orientation.w;

    siny_cosp = 2 * (encoder_ori_w * encoder_ori_z + encoder_ori_x * encoder_ori_y);
    cosy_cosp = 1 - 2 * (encoder_ori_y * encoder_ori_y + encoder_ori_z * encoder_ori_z);
    encoder_yaw = std::atan2(siny_cosp, cosy_cosp);
    encoder_vel_x = encoder_data->twist.twist.linear.x;
    encoder_ang_z = encoder_data->twist.twist.angular.z;
    pubi();
}

void pubi(){
        tf::TransformBroadcaster odom_brod;
        currentTime = ros::Time::now();

        x = encoder_weigth * encoder_x + laser_weigth * laser_x;
        y = encoder_weigth * encoder_y + laser_weigth * laser_y;

        ori = encoder_weigth * encoder_yaw + laser_weigth * laser_yaw;
        vel_x = encoder_weigth * encoder_vel_x + laser_weigth * laser_vel_x;
        ang_z = encoder_weigth * encoder_ang_z + laser_weigth * laser_ang_z;
      //  ROS_INFO("X: [%f]", x);
       // ROS_INFO("Y: [%f]", y);
      //  ROS_INFO("Heading: [%f]", ori);

        geometry_msgs::Quaternion quaternion = tf::createQuaternionMsgFromYaw(ori);

        geometry_msgs::TransformStamped transform;

        transform.header.stamp = currentTime;
        transform.header.frame_id = "odom_merge";
        transform.child_frame_id = "base_footprint";
        transform.transform.translation.x = x;
        transform.transform.translation.y = y;
        transform.transform.translation.z = 0.0;
        transform.transform.rotation = quaternion;

        odom_brod.sendTransform(transform);
        odom.header.stamp = currentTime;
        odom.header.frame_id = "odom_merge";
        odom.child_frame_id = "base_link";
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = quaternion;
        odom.twist.twist.linear.x = vel_x;
        odom.twist.twist.linear.y = 0;
        odom.twist.twist.angular.z = ang_z;

        pub.publish(odom);

    }
};

int main(int argc, char** argv){
    ros::init(argc, argv, "test");
    //ros::NodeHandle nh;
    test T;
    T.subscriber();
    T.publisher();
    //ros::Subscriber sub=nh.subscribe("floater",10,&bim::halo, &T);
    ros::spin();
    return 0;
}
