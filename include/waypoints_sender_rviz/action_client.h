#pragma once 
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <thread>
#include <memory>

#include <actionlib/client/simple_action_client.h>
#include <bebop2_controller/WaypointsActionGoal.h>
#include <bebop2_controller/WaypointsAction.h>
#include <waypoints_sender_rviz/path_generator.h>

class WaypointsActionGoal
{
public:
    WaypointsActionGoal() 
    {
        
       
    };
    ~WaypointsActionGoal() = default;

    void set(const std::string& drone_name, int goal_couner, int gear, bool orientation)  
    {
        action_name_ = drone_name + "/waypoint_action";
        goalCounter_ = goal_couner;
        gearCounter_ = gear;
        orientation_ = orientation;

        ac_ = std::make_unique<actionlib::SimpleActionClient<bebop2_controller::WaypointsAction>>(action_name_, true);
        action_thread_ = std::make_unique<std::thread>(&WaypointsActionGoal::run, this);
        action_thread_->detach();
    }
    
    bool isAlive()const 
    {
        return isAlive_;
    }
    
    
    void run()
    {
        ROS_INFO_STREAM("Action client is running for " << action_name_);
        
        ROS_INFO("Waiting for the action server to start.");
        ac_->waitForServer();
        isAlive_ = true;

        auto Eight = Eight::PathGenerator();
        Eight.setNumPoints(150);
        Eight.setPathScale(3.5, 1.5);
        auto path = Eight.generatePath(0.0);
        setPath(path);
        setGoalID(goalCounter_);
        
        
        auto goal = generateGoal();
        ac_->sendGoal(goal.goal);
        ROS_INFO("Sent the path to the action server.");
        ac_->waitForResult();
        ROS_INFO("Action server has finished.");
        isAlive_ = false;
    }
    
    void setGoal(const bebop2_controller::WaypointsActionGoal& goal)
    {
        goal_ = goal;
    }

    bebop2_controller::WaypointsActionGoal getGoal()
    {
        return goal_;
    }

    void setPath(const std::vector<std::array<float, 3>>& path)
    {
        goal_.goal.waypoints.clear();
        for (auto& point : path)
        {
            geometry_msgs::Point wp;
            wp.x = point[0];
            wp.y = point[1];
            wp.z = point[2];
            goal_.goal.waypoints.push_back(wp);
        }

        goal_.goal.method = 2;
        goal_.goal.gear = gearCounter_;
        goal_.goal.orinetation_control = orientation_;
    }
  

    void setGoalID(int id)
    {
        goal_.goal_id.id = id;
        goal_.goal_id.stamp = ros::Time::now();
    }

    bebop2_controller::WaypointsActionGoal generateGoal()
    {
        return goal_;
    }




private:
    bool isAlive_{false};
    std::unique_ptr<std::thread> action_thread_;
    int goalCounter_{0};
    int gearCounter_{0};
    bool orientation_{false};
    bebop2_controller::WaypointsActionGoal goal_;
    std::string action_name_{"waypoint_action"};
    std::unique_ptr<actionlib::SimpleActionClient<bebop2_controller::WaypointsAction>> ac_;

};