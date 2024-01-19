#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>

class ViconStateEstimator
{
public:
    ViconStateEstimator(ros::NodeHandle& nh):nh_(nh)
    {
        pub_ = nh_.advertise<nav_msgs::Odometry>("apriltag/state", 10);
    }
    void vicon_callback(const geometry_msgs::TransformStampedConstPtr& msg)
    {
        nav_msgs::Odometry odom;
        odom.header.stamp = ros::Time::now();
        odom.header.frame_id = "map";

        odom.pose.pose.position.x = msg->transform.translation.x;
        odom.pose.pose.position.y = msg->transform.translation.y;
        odom.pose.pose.position.z = msg->transform.translation.z;

        odom.pose.pose.orientation.x = msg->transform.rotation.x;
        odom.pose.pose.orientation.y = msg->transform.rotation.y;
        odom.pose.pose.orientation.z = msg->transform.rotation.z;
        odom.pose.pose.orientation.w = msg->transform.rotation.w;

        pub_.publish(odom);
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher pub_; 
};


int main(int argc, char* argv[])
{
    ros::init(argc, argv, "bebop2_vicon_node");
    ROS_INFO("bebop2_vicon_node INITIALIZED!");
    ros::NodeHandle nh; 
    ViconStateEstimator vicon(nh);
    ros::Subscriber sub = nh.subscribe("/vicon/bebop2/bebop2", 10, &ViconStateEstimator::vicon_callback, &vicon);
    ros::spin();

    return 0;
}
