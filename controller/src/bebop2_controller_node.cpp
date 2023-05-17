//
// Created by airlab on 5/16/23.
//
#include <ros/ros.h>
#include "JoystickInterface.h"
#include "DroneController.h"
int main(int argc, char* argv[])
{
    ros::init(argc, argv, "bebop2_controller");

    bebop2::JoystickInterface joyInterface;
    bebop2::DroneController droneController;

    double dt;
    ros::param::get("/dt", dt);

    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::Rate rate(int(1 / dt ));
    while (ros::ok())
    {
        auto currentMode = joyInterface.getButtonState();
        if(currentMode == bebop2::CONTROL)
        {
            auto setpoint = joyInterface.getSetpoint();
            droneController.sendControl(setpoint);
        }
        rate.sleep();
    }


    return 0;
}