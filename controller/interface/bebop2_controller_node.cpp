// Created by roboticslab on 12/13/22.
//
#include "ros/ros.h"
#include <iostream>
#include "../LQR/quad_lqg.h"
#include "../PID/quad_pids.h"
#include "ControllerInterface.h"


int main(int argc, char* argv[])
{
    ros::init(argc, argv, "airlib_bebop2");
    ROS_INFO("airlib-bebop2 INITIALIZED!");
    ros::NodeHandle nh;

//    std::vector<double> gains;
    double dt;
    ros::param::get("/dt", dt);

    std::vector<double>gains;
    std::string controllerType;
    ros::param::get("/controller", controllerType);

    std::shared_ptr<ControllerBase> controller;
    if(controllerType == "lqr")
    {
        ROS_INFO_STREAM("LQR controller selected");
        ros::param::get("/lqr_gains", gains);
        controller = std::make_shared<controller::quad_lqg>(gains, dt);

    } else
    {
        ROS_INFO_STREAM("PID controller selected");
        ros::param::get("/pid_gains", gains);
        controller = std::make_shared<controller::quad_pids>(gains, dt);

    }

    bebop2::ControllerInterface interface(nh, controller);

    ros::AsyncSpinner spinner(4);
    spinner.start();

    ros::waitForShutdown();

    return 0;
}
