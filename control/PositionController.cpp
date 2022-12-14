//
// Created by roboticslab on 12/14/22.
//

#include "PositionController.h"

bebop2::PositionController::PositionController(const StatePtr &stateEstimation) : stateEstimation_(stateEstimation) {
    timer_ = nh_.createTimer(ros::Duration(0.03), &bebop2::PositionController::control_loop, this);
}

void bebop2::PositionController::control_loop(const ros::TimerEvent &event) {
    //get current state
    auto current = stateEstimation_->getPosition();
    if (current.tagName.empty())
        return;

    if(buttonState_ != CONTROL)
        setPoints_ = current;

    if(buttonState_ == HOVER)
    {
        ROS_INFO_STREAM("Hovering Mode Selected");
        buttonState_ = CONTROL;
    }
}
