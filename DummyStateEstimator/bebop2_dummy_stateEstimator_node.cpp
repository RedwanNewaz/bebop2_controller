//
// Created by airlab on 6/1/23.
//
#include <ros/ros.h>
#include "DummyState.h"
#include "nav_msgs/Odometry.h"
#include "../Core/StateViz.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "dummy_state_estimator");
    ROS_INFO("Dummy State Estimator Started");
    ros::NodeHandle nh;
    auto pub = nh.advertise<nav_msgs::Odometry>("/bebop/ekf_state", 10);

    bebop2::DummyState dummyState(nh, false);
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::Rate r(100);

    nav_msgs::Odometry odom;
    bebop2::StateViz viz;


    while(ros::ok())
    {
        if (!dummyState.empty())
        {
            auto state = dummyState.getState();
            auto pos = state.getOrigin();
            odom.header.frame_id = "map";
            odom.header.stamp = ros::Time::now();
            odom.pose.pose.position.x = pos.x();
            odom.pose.pose.position.y = pos.y();
            odom.pose.pose.position.z = pos.z();
            odom.pose.pose.orientation.x = odom.pose.pose.orientation.y = odom.pose.pose.orientation.z = 0;
            odom.pose.pose.orientation.w = 1;
            viz.setDrone(state);
            pub.publish(odom);
        }
        r.sleep();
    }
    return 0;
}