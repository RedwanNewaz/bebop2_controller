//
// Created by airlab on 10/2/23.
//

#ifndef BEBOP2_CONTROLLER_P2PNAV_H
#define BEBOP2_CONTROLLER_P2PNAV_H

#include <string>
#include <memory>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include "bebop2_controller/SetpointsAction.h"
#include <filesystem>
#include <chrono>
#include "rapidcsv.h"
#include "airlib/control/controller.h"
#include "trajectory_planner.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>

class P2PNav {
    using WPI = std::unique_ptr<waypoint_trajectory_interface>;
    using MQ = std::shared_ptr<MessageQueue>;
public:
    P2PNav(std::string name);

    void preemptCB();

    void executeCB();

    void execute(const bebop2_controller::SetpointsGoalConstPtr &goal);

private:
    // Declare private member variables and methods here
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Subscriber state_sub_;
    std::string action_name_;
    actionlib::SimpleActionServer<bebop2_controller::SetpointsAction> as_;
    bebop2_controller::SetpointsFeedback feedback_;
    bebop2_controller::SetpointsResult result_;


    std::vector<double> current_state_;
    double max_vel, max_acc;
    bool force_terminate_, isAlive_, isStateValid_;
    std::mutex mtx_;
    std::condition_variable cv_;
    enum PlannerType
    {
        CV = 0,
        JERK,
        SNAP
    };

protected:

    void control_loop(const std::string& selected_planner,  MQ messageQueue);

    void state_callback(const nav_msgs::Odometry::ConstPtr &msg);
};


#endif //BEBOP2_CONTROLLER_P2PNAV_H
