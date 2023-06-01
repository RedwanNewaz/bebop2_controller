//
// Created by airlab on 5/17/23.
//

#ifndef BEBOP2_CONTROLLER_NODELET_JOYSTICKINTERFACE_H
#define BEBOP2_CONTROLLER_NODELET_JOYSTICKINTERFACE_H

#include <ros/ros.h>
#include "sensor_msgs/Joy.h"
#include "std_msgs/Empty.h"
#include <mutex>
#include <memory>
#include <geometry_msgs/Twist.h>
#include <cassert>
#include <functional>
#include <algorithm>
#include <numeric>
#include <nav_msgs/Odometry.h>
#include "../../Core/StateViz.h"

namespace bebop2 {
    const float STEP_INCR = 0.02;
    const float DEAD_ZONE = 0.3;
    const int X_AXIS_INDEX = 3;
    const int Y_AXIS_INDEX = 2;
    const int Z_AXIS_INDEX = 5;


    enum ButtonState{
        /// Setpoint can be freely moved with joystick. Sphere color blue
        IDLE = 0,
        /// Drone will takeoff from the ground
        TAKEOFF,
        /// Drone will land
        LAND,
        /// Set current location as a set point (for hover). Sphere color yellow
        ENGAGE,
        /// Start PID controller for the current setpoint. Sphere color cyan
        CONTROL
    };

    class JoystickInterface {

    public:
        JoystickInterface();
        std::vector<double> getSetpoint() const;
        ButtonState getButtonState() const;


    private:
        /// This ROS Handle attribute is used to create various publishers and subscribers which also deals with messages.
        ros::NodeHandle nh_;
        /// Locks some functions for safe operations
        std::mutex mu_;
        /// Array of four real numbers in which the first three numbers most commonly represented by x,y,z are vectors and the last number is w which represents the rotation of the robot about the vectors.
        std::vector<float>axes_values_;
        /// @brief Timer to call the joystick_timer_callback() function at a specific rate.
        /// @note The joystick_timer_callback() is invoked every 0.05 second.
        ros::Timer joystick_timer_;
        /// @brief Timer to call the control_loop() function at a specific rate.
        ros::Timer controller_timer_;
        /// @brief Unique Pointer that points to the address of StateViz class in the StateViz.h header file.
        std::unique_ptr<StateViz> viz_;
        /// Subscribes from sensors_msgs/joy and calls the joystick_callback() as a callback function.
        ros::Subscriber joystick_sub_;
        /// Subscribes bebop2 state from nav_msg/Odometry and calls the ekf_subscriber() as a callback function.
        ros::Subscriber state_sub_;
        /// Publishes the takeoff, landing and velocity information respectively.
        ros::Publisher drone_takeoff_pub_,  drone_land_pub_;
        /// @brief Co-ordinates of the points set by the controller.
        std::vector<double> setPoints_;
        /// @brief Applications applied to control the state of the robot using Joystick controller.
        ButtonState buttonState_;



    protected:
        /// @brief This function finds out the engaged button of the joystick i.e, checks the state of the button and
        /// and publishes the resulted behavior of the drone (Take Off or Land).
        /// It also updates the positon of drone which is also passed to RVIZ to update the position and path for visalization.
        /// @param msg reports the state of a joysticks axes and buttons.
        void joystick_callback(const sensor_msgs::Joy::ConstPtr& msg);
        /**
        * @brief Updates the setpoint of the drone when the axes of the joystick controller is changed or moved. As a result, it also updates the color and path of the drone for visualization.
        * @param event Creates a timer to call a function at a specific rate.
        */
        void joystick_timer_callback(const ros::TimerEvent& event);

        /// @brief Sets the position and rotation of the drone through the setpoints .
        /// @param state is used as an parameter to pass the setpoints.
        /// @return Returns the tf::Transform pose which is the new position and rotated angle of the robot.
        tf::Transform getStateVecToTransform(const std::vector<double>& state);


        void ekf_subscriber(const nav_msgs::Odometry::ConstPtr& msg);


    };

} // bebop2

#endif //BEBOP2_CONTROLLER_NODELET_JOYSTICKINTERFACE_H
