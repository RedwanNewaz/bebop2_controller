//
// Created by redwan on 10/1/23.
//

#ifndef BEBOP2_CONTROLLER_WAYPOINTCONTROLLER_H
#define BEBOP2_CONTROLLER_WAYPOINTCONTROLLER_H


#include <string>
#include <memory>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include "bebop2_controller/WaypointsAction.h"
#include <filesystem>
#include <chrono>
#include "rapidcsv.h"
#include "airlib/control/controller.h"
#include "trajectory_planner.h"

class WaypointController
{
    using WPI = std::unique_ptr<waypoint_trajectory_interface>;
    using MQ = std::shared_ptr<MessageQueue>;
public:
    WaypointController(std::string name);

    virtual ~WaypointController();

    void preemptCB();

    void executeCB();

    void execute(const bebop2_controller::WaypointsGoalConstPtr &goal);

protected:
    // Declare private member variables and methods here
    ros::NodeHandle nh_;
    std::string action_name_;
    actionlib::SimpleActionServer<bebop2_controller::WaypointsAction> as_;
    bebop2_controller::WaypointsFeedback feedback_;
    bebop2_controller::WaypointsResult result_;

    std::shared_ptr<bebop2::ControllerInterface> interface_;
    double max_vel, max_acc;
    bool force_terminate_, isAlive_;
    std::mutex mtx_;
    std::condition_variable cv_;
    enum PlannerType
    {
        CV = 0,
        JERK,
        SNAP
    };

protected:
    WAYPOINTS getPath(const std::string& path);
    WPI getPlanner(int method, MQ messageQueue, std::string& selected_planner);
    void control_loop(const std::string& selected_planner,  MQ messageQueue);
};

#endif //BEBOP2_CONTROLLER_WAYPOINTCONTROLLER_H
