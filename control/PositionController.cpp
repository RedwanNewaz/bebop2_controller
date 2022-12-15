//
// Created by roboticslab on 12/14/22.
//

#include "PositionController.h"

#include <utility>

bebop2::PositionController::PositionController( std::shared_ptr<StateEstimation> stateEstimation) : stateEstimation_(std::move(stateEstimation)) {
    timer_ = nh_.createTimer(ros::Duration(0.03), &bebop2::PositionController::control_loop, this);
}

void bebop2::PositionController::control_loop(const ros::TimerEvent &event) {
    //get current state
    auto current = stateEstimation_->getPosition();
    if (current.tagName.empty())
        return;

    if(buttonState_ == ENGAGE)
        setPoints_ = current;



}

void bebop2::PositionController::update_set_point(double dx, double dy, double dz) {

    if(setPoints_.tagName.empty())
        return;
    FieldLocation incr{"setpoint", dx, dy, dz};
    setPoints_ = setPoints_ + incr;

    tf::Transform transform;
    transform.setOrigin(tf::Vector3(setPoints_.x, setPoints_.y, setPoints_.z));

}
