//
// Created by redwan on 12/15/22.
//
/**
 * Controller Base has
 *  a) joystick controller to setup goal location, takeoff and landing commands
 *  b) visualization interface to show the result in rviz
 * Controller Base should take different type of controller
 *  e.g., i) PID controller
 *        ii) LQR controller
 * Controller Base should take a StatePtr and obtain robot state from there
 * Variable convention
 *  private variables end with _
 *  protected variables start with m_
 */
#ifndef BEBOP2_CONTROLLER_CONTROLLERBASE_H
#define BEBOP2_CONTROLLER_CONTROLLERBASE_H
#include <ros/ros.h>
#include "sensor_msgs/Joy.h"
#include "std_msgs/Empty.h"
#include <mutex>
#include <memory>
#include <geometry_msgs/Twist.h>
#include <cassert>
#include <functional>
#include <algorithm>
#include "ControlViz.h"

#include "../localization/StateObserver.h"
#include "../localization/StateObserver.cpp"

namespace bebop2
{
    const float STEP_INCR = 0.02;
    const float DEAD_ZONE = 0.3;
    const int X_AXIS_INDEX = 3;
    const int Y_AXIS_INDEX = 2;
    const int Z_AXIS_INDEX = 5;
    const int NUM_CONTROLS = 4;



    template<class Sensor, class Filter>
    class ControllerBase{
    public:


        explicit ControllerBase(StatePtr<Sensor, Filter> mGetState, ros::NodeHandle& nh);


    private:
        std::vector<float>axes_values_;
        ros::Timer joystick_timer_;
        ros::Timer controller_timer_;
        std::unique_ptr<ControlViz> viz_;
        ros::Subscriber joystick_sub_;
        ros::Publisher drone_takeoff_pub_,  drone_land_pub_,cmd_vel_pub_;
        std::vector<double> setPoints_;
    protected:
        ros::NodeHandle m_nh; // if you need to create some publisher and subscriber
        std::mutex m_mu; // if you need to lock some function for safe operations
        enum ButtonState{
            IDLE = 0, // setpoint can be freely moved with joystick. Sphere color blue
            TAKEOFF, // drone will takeoff from the ground
            LAND, //drone will land
            ENGAGE, // set current location as a set point (for hover). Sphere color yellow
            CONTROL // start PID controller for the current setpoint. Sphere color cyan
        }m_buttonState;

        // this function will give robot state
        StatePtr<Sensor, Filter> m_get_state;

        // controller param
        double m_dt; // sample time
        double m_goal_thres; // goal region threshold



    private:
        void joystick_callback(const sensor_msgs::Joy::ConstPtr& msg);
        void joystick_timer_callback(const ros::TimerEvent& event);

        void control_loop(const ros::TimerEvent& event);
        tf::Transform getStateVecToTransform(const std::vector<double>& state);

    protected:
        void publish_cmd_vel(const std::vector<double>&U);
        double goal_distance(const std::vector<double>& X, const std::vector<double>& setPoints);
        virtual void compute_control(const std::vector<double>& X, const std::vector<double>& setPoints, std::vector<double>& control) = 0;

 };
}


#endif //BEBOP2_CONTROLLER_CONTROLLERBASE_H
