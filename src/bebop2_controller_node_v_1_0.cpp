//
// Created by redwan on 3/5/23.
//
//
// Created by roboticslab on 12/13/22.
//
#include "ros/ros.h"
#include <iostream>
#include "airlib/control/controller.h"


int main(int argc, char* argv[])
{
    ros::init(argc, argv, "airlib_bebop2");
    ROS_INFO("airlib-bebop2 INITIALIZED!");
    ros::NodeHandle nh;

    std::vector<double> gains;
    double dt;
    ros::param::get("/pid_gains", gains);
    ros::param::get("/dt", dt);

    auto controller = std::make_shared<controller::quad_pids>(gains, dt);
    bebop2::ControllerInterface interface(nh, controller);

    ros::AsyncSpinner spinner(4);
    spinner.start();

    ros::waitForShutdown();

    return 0;
}
