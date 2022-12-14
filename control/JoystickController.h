//
// Created by roboticslab on 12/13/22.
//

#ifndef BEBOP2_CONTROLLER_JOYSTICKCONTROLLER_H
#define BEBOP2_CONTROLLER_JOYSTICKCONTROLLER_H
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/Empty.h"
#include <mutex>

namespace bebop2{
    enum ButtonState{
        EMERGENCY = 0,
        TAKEOFF,
        LAND,
        HOVER,
        CONTROL
    };
    class JoystickController {
    public:
        JoystickController();
        void joystick_callback(const sensor_msgs::Joy::ConstPtr& msg);

    protected:
        ButtonState buttonState_;
        ros::NodeHandle nh_;
        ros::Subscriber joystick_sub_;
        ros::Publisher drone_takeoff_pub_,  drone_land_pub_;
        std::mutex mu_;

    private:
        double step_incr_, dead_zone_;
    };
}


#endif //BEBOP2_CONTROLLER_JOYSTICKCONTROLLER_H
