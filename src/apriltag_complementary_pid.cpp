//
// Created by roboticslab on 12/13/22.
//
#include "ros/ros.h"
#include <iostream>
#include "airlib/control/QuadControllerPID.h"
#include "airlib/localization/Sensors/ApriltagLandmarksExtended.h"


int main(int argc, char* argv[])
{
    ros::init(argc, argv, "airlib_bebop2");
    ROS_INFO("BEBOP2 CONTROLLER INITIALIZED!");
    ros::NodeHandle nh;
    double alpha;
    ros::param::get("~alpha", alpha);

    auto stateFilter = std::make_shared<ComplementaryFilter>(alpha);
    auto stateSensor = std::make_shared<ApriltagLandmarksExtended>(nh);
    auto stateObserver = std::make_shared<bebop2::StateObserver>(stateFilter, stateSensor);


    bebop2::QuadControllerPID controller(stateObserver, nh);
    const int THREAD_COUNT = 4;

    ros::AsyncSpinner spinner(THREAD_COUNT);
    spinner.start();
    ros::waitForShutdown();

//    ros::MultiThreadedSpinner spinner(THREAD_COUNT);
//    spinner.spin();
    return 0;
}