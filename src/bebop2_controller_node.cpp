//
// Created by roboticslab on 12/13/22.
//
#include "ros/ros.h"

#include <iostream>

#include "../localization/StateEstimation.h"
#include "../control/PositionController.h"


int main(int argc, char* argv[])
{
    ros::init(argc, argv, "bebop2_controller");
    ROS_INFO("BEBOP2 CONTROLLER INITIALIZED!");

    auto state = std::make_shared<StateEstimation>();
    bebop2::PositionController positionController(state);



    ros::MultiThreadedSpinner spinner(2);
    spinner.spin();

    return 0;
}