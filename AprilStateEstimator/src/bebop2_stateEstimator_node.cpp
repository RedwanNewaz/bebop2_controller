//
// Created by airlab on 5/16/23.
//
#include <ros/ros.h>
#include "StateEstimator.h"
#include "../../Core/StateViz.h"

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "bebop2_stateEstimator");
    Alse::StateEstimator stateEstimator;
    bebop2::StateViz viz;


    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::Rate rate(100);
    while (ros::ok())
    {
        if(stateEstimator.isReady())
        {
            auto state = stateEstimator.getState();
            viz.setDrone(state);
        }
        rate.sleep();
    }
    return 0;
}