//
// Created by roboticslab on 12/14/22.
//

#include "PositionController.h"

#include <utility>

bebop2::PositionController::PositionController( std::shared_ptr<StateEstimation> stateEstimation) : stateEstimation_(std::move(stateEstimation)) {

    ros::param::get("~dt", dt_);
    ros::param::get("~goal_thres", goal_thres_);
    std::vector<double> gains;
    ros::param::get("~pid_gains", gains);
    set_gains(gains, m_quadController);
    timer_ = nh_.createTimer(ros::Duration(dt_), &bebop2::PositionController::control_loop, this);
}

void bebop2::PositionController::control_loop(const ros::TimerEvent &event) {
    //get current state
    auto current = stateEstimation_->getPosition();
    if (current.tagName.empty())
        return;


    if(buttonState_ == ENGAGE)
    {
        // set setpoint at current position
        setPoints_ = current;
        update_set_point(0, 0, 0);
    }

    if(buttonState_ == CONTROL)
    {
        // actively control position
        std::vector<double>X{current.x,current.y,current.z,0}, Xg{setPoints_.x,setPoints_.y,setPoints_.z,0.0}, U(NUM_CONTROLLER);
        if(!compute_control(X, Xg, m_quadController, U))
        {
            ROS_INFO("[PositionController] vx = %lf, vy = %lf, vz = %lf", U[0], U[1], U[2]);
            publish_cmd_vel(U);
        }

    }

    // show drone state
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(current.x, current.y, current.z));
    viz_->setDrone(transform);



}

void bebop2::PositionController::update_set_point(double dx, double dy, double dz) {

    if(setPoints_.tagName.empty())
        return;
    FieldLocation incr{"setpoint", dx, dy, dz};
    setPoints_ = setPoints_ + incr;

    tf::Transform transform;
    transform.setOrigin(tf::Vector3(setPoints_.x, setPoints_.y, setPoints_.z));
    update_setpoint_viz(transform);
}

void bebop2::PositionController::set_gains(const std::vector<double> &gains, bebop2::PID *controller) {
    const double MAX_OUT = 1;
    const double MIN_OUT = -1;

    for (int i = 0; i < NUM_CONTROLLER; ++i) {
        int j = i * NUM_GAINS;
        controller[i].init(dt_, MAX_OUT, MIN_OUT, gains[j], gains[j + 1], gains[j + 2]);
    }
}

double bebop2::PositionController::goal_distance(const std::vector<double> &X, const std::vector<double> &setPoints) {
    double error = 0;
    for (int i = 0; i < X.size(); ++i) {
        double e = X[i] - setPoints[i];
        error += e * e;
    }
    return sqrt(error);
}

bool bebop2::PositionController::compute_control(const std::vector<double> &X, const std::vector<double> &setPoints,
                                                 bebop2::PID *controller, std::vector<double> &control) {
    std::lock_guard<std::mutex> lk(mu_);
    auto dist = goal_distance(X, setPoints);
    if(dist < goal_thres_)
    {
        ROS_INFO("[PositionController]: Hovering radius = %lf", dist );
        return false;
    }
    for (int i = 0; i < NUM_CONTROLLER; ++i) {
        control[i] = controller[i].calculate(setPoints[i], X[i]);
    }
    return true;
}
