// Created by roboticslab on 12/13/22.
//
#include "ros/ros.h"
#include <iostream>
#include "../LQR/quad_lqg.h"
#include "ControllerInterface.h"


int main(int argc, char* argv[])
{
    ros::init(argc, argv, "airlib_bebop2");
    ROS_INFO("airlib-bebop2 INITIALIZED!");
    ros::NodeHandle nh;

//    std::vector<double> gains;
    double dt;
//    ros::param::get("/pid_gains", gains);
    ros::param::get("/dt", dt);

    std::vector<double>gains{
            0.075, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0,
            0.0, 0.075, 0.0, 0.0, 0.0, 0.00075, 0.0, 0.0,
            0.0, 0.0, 0.085, 0.0, 0.0, 0.0, 0.001, 0.0,
            0.0, 0.0, 0.0, 0.08, 0.0, 0.0, 0.0, 0.002,
            0.0, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.00075, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.001, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.002
    };

    auto controller = std::make_shared<controller::quad_lqg>(gains, dt);
    bebop2::ControllerInterface interface(nh, controller);

    ros::AsyncSpinner spinner(4);
    spinner.start();

    ros::waitForShutdown();

    return 0;
}
