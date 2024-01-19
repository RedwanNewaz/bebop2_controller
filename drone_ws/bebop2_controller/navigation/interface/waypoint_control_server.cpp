#include <ros/ros.h>
#include "../waypoints/WaypointController.h"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "waypoint_controller");
    WaypointController controller("waypoint_action"); // Use a meaningful name for your action server

    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::waitForShutdown();
    return 0;
}
