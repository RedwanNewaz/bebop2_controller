//
// Created by roboticslab on 12/13/22.
//
#include "ros/ros.h"
#include <iostream>
#include "airlib/control/QuadControllerPID.h"
#include "airlib/localization/Sensors/ApriltagLandmarksExtended.h"
#include "airlib/localization/Filters/RobotLocalization.h"

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "airlib_bebop2");
    ROS_INFO("BEBOP2 CONTROLLER INITIALIZED!");
    ros::NodeHandle nh;
    double alpha;
    ros::param::get("~alpha", alpha);


    auto stateFilter = std::make_shared<bebop2::RobotLocalization>(nh);
    auto stateSensor = std::make_shared<ApriltagLandmarksExtended>(nh);
    auto stateObserver = std::make_shared<bebop2::StateObserver>(stateFilter, stateSensor);

    // use cmd_vel to update state
    auto cmd_sub = nh.subscribe("cmd_vel", 1, &bebop2::RobotLocalization::update_cmd, stateFilter.get());


    bebop2::QuadControllerPID controller(stateObserver, nh);
    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::waitForShutdown();

//    ros::MultiThreadedSpinner spinner(2);
//    spinner.spin();
    return 0;
}