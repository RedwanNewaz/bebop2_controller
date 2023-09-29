//
// Created by redwan on 3/5/23.
//
//
// Created by roboticslab on 12/13/22.
//
#include "ros/ros.h"
#include <iostream>
#include "airlib/control/QuadControllerPID.h"
#include "airlib/localization/Sensors/ApriltagLandmarksExtended.h"
#include "airlib/localization/Filters/RobotLocalization.h"
#include <nav_msgs/Odometry.h>
#include <future>
#include <thread>

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "airlib_bebop2");
    ROS_INFO("airlib-bebop2 INITIALIZED!");
    ros::NodeHandle nh;
    double alpha;
    ros::param::get("/alpha", alpha);
    auto pub = nh.advertise<nav_msgs::Odometry>("apriltag/state", 10);

    auto stateSensor = std::make_shared<ApriltagLandmarks>(nh);
    auto stateFilter = std::make_shared<ComplementaryFilter>(alpha);
    auto stateObserver = std::make_shared<bebop2::StateObserver>(stateFilter, stateSensor);


    ros::AsyncSpinner spinner(4);
    spinner.start();

    std::thread stateThread = std::thread ([&]{
        try {
            // code that may throw
            while(ros::ok())
            {
                std::promise<std::vector<double>> promise;
                stateObserver->getState(std::ref(promise));
                auto xEst = promise.get_future();
                auto state = xEst.get();
                ROS_INFO("[State] (%lf, %lf, %lf, %lf)", state[0], state[1], state[2], state[3]);

                nav_msgs::Odometry odom;
                odom.header.stamp = ros::Time::now();
                odom.header.frame_id = "map";

                odom.pose.pose.position.x = state[0];
                odom.pose.pose.position.y = state[1];
                odom.pose.pose.position.z = state[2];

                tf::Quaternion q;
                q.setRPY(0, 0, state[3]);
                odom.pose.pose.orientation.x = q.x();
                odom.pose.pose.orientation.y = q.y();
                odom.pose.pose.orientation.z = q.z();
                odom.pose.pose.orientation.w = q.w();

                pub.publish(odom);

            }
        } catch(...) {

        }
    });

    ros::waitForShutdown();

    return 0;
}
