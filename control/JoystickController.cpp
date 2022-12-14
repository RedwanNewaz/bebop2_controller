//
// Created by roboticslab on 12/13/22.
//

#include "JoystickController.h"

bebop2::JoystickController::JoystickController() {

    joystick_sub_ = nh_.subscribe("/joy", 10, &bebop2::JoystickController::joystick_callback, this);
    drone_takeoff_pub_ = nh_.advertise<std_msgs::Empty>("", 1);
    drone_land_pub_ = nh_.advertise<std_msgs::Empty>("", 1);
}

void bebop2::JoystickController::joystick_callback(const sensor_msgs::Joy::ConstPtr &msg) {
    std::lock_guard<std::mutex> lk(mu_);
    auto it = std::find(msg->buttons.begin(), msg->buttons.end(), 1);
    if(it != msg->buttons.end())
    {
        int index = std::distance(msg->buttons.begin(), it);
        buttonState_ = static_cast<ButtonState>(index);

        std_msgs::Empty empty;
        switch (buttonState_) {
            case TAKEOFF: drone_takeoff_pub_.publish(empty); ROS_INFO("[Button] pressed = %d -> TAKE OFF", buttonState_);break;
            case LAND: drone_land_pub_.publish(empty); ROS_INFO("[Button] pressed = %d -> LAND", buttonState_); break;
        }
    }


}
