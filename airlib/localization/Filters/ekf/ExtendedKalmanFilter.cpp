//
// Created by Redwan Newaz on 12/29/22.
//

#include "airlib/localization/Filters/ExtendedKalmanFilter.h"

using namespace bebop2;

ExtendedKalmanFilter::ExtendedKalmanFilter(const std::vector<double>& sigma_pos, double dt, int state_dim)
:DT(dt), STATE_DIM(state_dim), CONTROL_DIM(STATE_DIM)
{
    
    Q_.resize(STATE_DIM, STATE_DIM);
    R_.resize(STATE_DIM, STATE_DIM);

    for (int i = 0; i < STATE_DIM; ++i) {
        Q_(i, i) = sigma_pos[i];
        R_(i, i) = 1;
    }

    X_.resize(STATE_DIM);
    xEst_.resize(STATE_DIM);
    PEst_.resize(STATE_DIM, STATE_DIM);
    PEst_.setIdentity();


}

void ExtendedKalmanFilter::operator()(std::vector<double> &state) {

    if(!state.empty())
        state.clear();

    std::copy(X_.begin(), X_.end(), std::back_inserter(state));

}


Eigen::VectorXd ExtendedKalmanFilter::motion_model(const Eigen::VectorXd &x, const Eigen::VectorXd &u) {
    Eigen::MatrixXd F_;
    F_.resize(STATE_DIM, STATE_DIM);
    F_<<1.0,   0,   0,   0,
            0, 1.0,   0,   0,
            0,   0, 1.0,   0,
            0,   0,   0, 1.0;

    Eigen::MatrixXd B_;
    B_.resize(STATE_DIM, STATE_DIM);
    B_<<    DT,  0,   0,   0,
            DT,  0,   0,   0,
            DT,  0,   0,   0,
            DT,  0,   0,   0;
    
    return F_ * x + B_ * u;
}

Eigen::MatrixXd ExtendedKalmanFilter::jacobF(const Eigen::VectorXd &x, const Eigen::VectorXd &u) {
    Eigen::MatrixXd jF_ = Eigen::Matrix4d::Identity();
//    float yaw = x(2);
//    float v = u(0);
//    jF_(0,2) = -DT * v * std::sin(yaw);
//    jF_(0,3) = DT * std::cos(yaw);
//    jF_(1,2) = DT * v * std::cos(yaw);
//    jF_(1,3) = DT * std::sin(yaw);
    return jF_;
}

Eigen::MatrixXd ExtendedKalmanFilter::jacobH() const {
    //# Jacobian of Observation Model
    Eigen::Matrix4d jH;
    jH.setIdentity();
    return jH;
}

Eigen::VectorXd ExtendedKalmanFilter::observation_model(const Eigen::VectorXd &x) {
    Eigen::Matrix4d H_;
    H_.setIdentity();
    return H_ * x;
}

void ExtendedKalmanFilter::internal_update(const Eigen::VectorXd &z, const Eigen::VectorXd &u) {
    Eigen::VectorXd xPred = motion_model(xEst_, u_);
    Eigen::MatrixXd jF = jacobF(xPred, u_);
    Eigen::MatrixXd PPred = jF * PEst_ * jF.transpose() + Q_;

    Eigen::MatrixXd jH = jacobH();
    Eigen::VectorXd zPred = observation_model(xPred);
    Eigen::VectorXd y = z - zPred;
    Eigen::MatrixXd S = jH * PPred * jH.transpose() + R_;
    Eigen::MatrixXd K = PPred * jH.transpose() * S.inverse();
    xEst_ = xPred + K * y;
    PEst_ = (Eigen::Matrix4d::Identity() - K * jH) * PPred;
}

void ExtendedKalmanFilter::update_cmd(const geometry_msgs::Twist::ConstPtr& cmd)     {
    u_.resize(CONTROL_DIM);
    u_ <<   cmd->linear.x,
            cmd->linear.y,
            cmd->linear.z,
            cmd->angular.z;
}

void ExtendedKalmanFilter::update(const std::vector<double>& obs, std::vector<double>& result)
{
    Eigen::VectorXd z(obs.size());
    for (int i = 0; i < obs.size(); ++i) {
        z(i) = obs[i];
    }
    internal_update(z, u_);
    for (int j = 0; j < STATE_DIM; ++j) {
        result[j] = xEst_(j);
    }
}

void ExtendedKalmanFilter::init(const std::vector<double> &X0) {
    for (int j = 0; j < STATE_DIM; ++j) {
        xEst_(j) = X_[j] = X0[j];
    }
}
