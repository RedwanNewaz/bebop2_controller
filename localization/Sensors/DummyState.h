#pragma once 

/** Dummy state will create a state space repersentation for quadrotor. 
 * The key idea is to initialize a state at (0, 0, 0, 0) and take joystick command to update state. 
 * To capture realistic behavior add some gaussian white noise to the state. 
 * When bebop2 takeoff from the ground it reaches 1m above from the ground origin and when it lands then z axis value is 0. 
 * This behavior can be achieved by simply subscribing takeoff and land messages.
 * To change bebop coordinates in x and y direction subscribe cmd_vel topic and update the state based on the cmd_vel value. 
 * cmd_vel is a geometry_msg/Twist msg which provides 4 control inputs, i.e., linear.x, linear.y, linear.z, angular.z
 * Here, we need to make a assumption that sampling time DT is a constant value.  
 * Finally to integrate with our main framework, we need a public method void operator()(std::vector<double>& state) that can populate a requested state vector; 
 * 
 */


#include <iostream>
#include <vector>
#include <random>
#include <math.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>

namespace bebop2{
    class DummyState{

    public:
        DummyState(ros::NodeHandle& nh);
        void operator()(std::vector<double>& state);
        bool empty();

    private:
        ros::NodeHandle nh_;
        std::vector<double> states_;
        ros::Subscriber sub_takeoff_, sub_land_, sub_cmd_vel_;
        std::random_device rd_;
        
        const size_t STATE_DIM = 4;
        const double DT = 0.03;
        const double NOISE = 0.2; 
    protected:
        void takeoff_callback(const std_msgs::Empty::ConstPtr& msg);
        void land_callback(const std_msgs::Empty::ConstPtr& msg);
        void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg);

    };
}

