//
// Created by airlab on 10/2/23.
//
#include <ros/ros.h>
#include "../ptp_move/P2PNav.h"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "move_p2p_controller");
    P2PNav controller("move_p2p"); // Use a meaningful name for your action server

    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::waitForShutdown();
    return 0;
}
