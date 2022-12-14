//
// Created by roboticslab on 12/13/22.
//
#include "ros/ros.h"

#include <iostream>

#include "../control/JoystickController.h"
#include "../localization/StateEstimation.h"


int main(int argc, char* argv[])
{
    ros::init(argc, argv, "bebop2_controller");
    ROS_INFO("BEBOP2 CONTROLLER INITIALIZED!");



    bebop2::JoystickController controller;
    StateEstimation stateEstimation;


    ros::MultiThreadedSpinner spinner(2);
    spinner.spin();

    return 0;
}