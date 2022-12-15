//
// Created by roboticslab on 12/13/22.
//

#include "JoystickController.h"

bebop2::JoystickController::JoystickController() {

    joystick_sub_ = nh_.subscribe("/joy", 10, &bebop2::JoystickController::joystick_callback, this);
    drone_takeoff_pub_ = nh_.advertise<std_msgs::Empty>("", 1);
    drone_land_pub_ = nh_.advertise<std_msgs::Empty>("", 1);
    viz_ = std::make_unique<ControlViz>(nh_);
    buttonState_ = ENGAGE;
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
            case TAKEOFF: {
                drone_takeoff_pub_.publish(empty); ROS_INFO("[Button] pressed = %d -> TAKE OFF", buttonState_);break;
                buttonState_ = IDLE;
            }
            case LAND: drone_land_pub_.publish(empty); ROS_INFO("[Button] pressed = %d -> LAND", buttonState_); break;
        }
    }

    // get axis
    set_goalpoint(msg->axes);


}

void bebop2::JoystickController::set_goalpoint(const std::vector<float> &axes) {

    auto filter = [&](float val){
        float sign = val >= 0 ? 1 : -1;
        return abs(val) > DEAD_ZONE ? sign * STEP_INCR : 0;
    };

    float dz = filter(axes[Z_AXIS_INDEX]);
    float dx = filter(axes[X_AXIS_INDEX]);
    float dy = filter(axes[Y_AXIS_INDEX]);
    update_set_point(dx, -dy, dz);

}

void bebop2::JoystickController::update_setpoint_viz(const tf::Transform &pose) {
    switch (buttonState_) {
        case TAKEOFF: viz_->update(pose, GREEN); break;
        case IDLE:    viz_->update(pose, BLUE); break;
        case ENGAGE:  viz_->update(pose, YELLOW);  break;
        case CONTROL: viz_->update(pose, CYAN);   break;
        case LAND:    viz_->update(pose, RED);    break;
    }
}
