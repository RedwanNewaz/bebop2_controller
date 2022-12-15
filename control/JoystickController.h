//
// Created by roboticslab on 12/13/22.
//

#ifndef BEBOP2_CONTROLLER_JOYSTICKCONTROLLER_H
#define BEBOP2_CONTROLLER_JOYSTICKCONTROLLER_H
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/Empty.h"
#include <mutex>
#include <memory>
#include <geometry_msgs/Twist.h>
#include <cassert>
#include "ControlViz.h"

namespace bebop2{
    enum ButtonState{
        IDLE = 0, // setpoint can be freely moved with joystick. Sphere color blue
        TAKEOFF, // drone will takeoff from the ground
        LAND, //drone will land
        ENGAGE, // set current location as a set point (for hover). Sphere color yellow
        CONTROL // start PID controller for the current setpoint. Sphere color cyan
    };
    class JoystickController {
    public:
        JoystickController();
        void joystick_callback(const sensor_msgs::Joy::ConstPtr& msg);

    protected:
        ButtonState buttonState_;
        ros::NodeHandle nh_;
        ros::Subscriber joystick_sub_;
        ros::Publisher drone_takeoff_pub_,  drone_land_pub_, cmd_vel_pub_;
        std::mutex mu_;
        std::unique_ptr<ControlViz> viz_;

        virtual void update_set_point(double dx, double dy, double dz) = 0;
        void update_setpoint_viz(const tf::Transform& pose);

        void publish_cmd_vel(const std::vector<double>&U);

    private:
        const float STEP_INCR = 0.01;
        const float DEAD_ZONE = 0.3;
        const int X_AXIS_INDEX = 3;
        const int Y_AXIS_INDEX = 4;
        const int Z_AXIS_INDEX = 1;

        void set_goalpoint(const std::vector<float>& axes);

    };
}


#endif //BEBOP2_CONTROLLER_JOYSTICKCONTROLLER_H
