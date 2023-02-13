#pragma once 

#include <iostream>
#include <vector>
#include <random>
#include <math.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include "SensorBase.h"
#include <queue>

namespace bebop2{
    /** 
     * Dummy state will create a state space repersentation for quadrotor. 
     * The key idea is to initialize a state at (0, 0, 0, 0) and take joystick command to update state. 
     * To capture realistic behavior add some gaussian white noise to the state. 
     * When bebop2 takeoff from the ground it reaches 1m above from the ground origin and when it lands then z axis value is 0. 
     * This behavior can be achieved by simply subscribing takeoff and land messages.
     * To change bebop coordinates in x and y direction subscribe cmd_vel topic and update the state based on the cmd_vel value. 
     * cmd_vel is a geometry_msg/Twist msg which provides 4 control inputs, i.e., linear.x, linear.y, linear.z, angular.z
     * Here, we need to make a assumption that sampling time DT is a constant value.  
     * Finally to integrate with our main framework, we need a public method void operator()(std::vector<double>& state) that can populate a requested state vector; 
     */
    class DummyState: public SensorBase{

    public:
        /**
         * @brief Construct a new Dummy State object
         * To simulate desired behavior from a sensor so that we can develop accurate control module 
         * @param nh ros NodeHandle to deal with various messages
         * @param noisy_reading an option to make a realistic sensor reading: if true white noise will be added to the output 
         */
        DummyState(ros::NodeHandle& nh, bool noisy_reading);
        void operator()(std::vector<double>& state);
        bool empty();

    private:
        ros::NodeHandle nh_;
        ros::Timer dummy_timer_;
        /// @brief internal states must contain at least four vairables [x, y, z, yaw] for the position control module
        std::vector<double> states_;
        /// @brief bebop_autonomy messages for basic maneuvers 
        ros::Subscriber sub_takeoff_, sub_land_, sub_cmd_vel_;
        /// @brief to generate Gaussian white noise we need random device
        std::random_device rd_;
        bool noisy_reading_;
        
        const size_t STATE_DIM = 4;
        /// @brief discrete sample time is determined by control frequence
        const double DT = 0.03; // 30 Hz
        /// @brief white noise amplitude 
        const double NOISE = 0.2;
        std::queue<std::vector<double>> queue_;
    protected:
        /**
         * @brief Joystic command to takeoff the bebop2 
         * @param msg  publish empty message to the topic ```takeoff```
        */
        void takeoff_callback(const std_msgs::Empty::ConstPtr& msg);
        /**
         * @brief @brief Joystic command to land the bebop2 
         * 
         * @param msg publish empty message to the topic ```land```
         */
        void land_callback(const std_msgs::Empty::ConstPtr& msg);
        /**
         * @brief Joystic command to move the bebop2 
         * 
         * @param msg publish Twist message to the topic ```cmd_vel``` 
         */
        void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg);

        void updateState(const ros::TimerEvent& event);

    };
}

