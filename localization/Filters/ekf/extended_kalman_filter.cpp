//
// Created by Redwan Newaz on 12/29/22.
//

#include "extended_kalman_filter.h"

extended_kalman_filter::extended_kalman_filter(const std::vector<double> &state, const std::vector<double>& sigma_pos, double dt, int state_dim) 
:DT(dt), STATE_DIM(state_dim), CONTROL_DIM(STATE_DIM)
{
    
    Q_.resize(STATE_DIM, STATE_DIM);
    R_.resize(STATE_DIM, STATE_DIM);

    for (int i = 0; i < STATE_DIM; ++i) {
        Q_(i, i) = sigma_pos[i];
        R_(i, i) = 1;
    }

    xEst_.resize(STATE_DIM);
    PEst_.resize(STATE_DIM, STATE_DIM);
    PEst_.setIdentity();

    for (int j = 0; j < STATE_DIM; ++j) {
        xEst_(j) = state[j];
    }
}

void extended_kalman_filter::operator()(std::vector<double> &state) {
    for (int j = 0; j < state.size(); ++j) {
        state[j] = xEst_(j);
    }

}


Eigen::VectorXd extended_kalman_filter::motion_model(const Eigen::VectorXd &x, const Eigen::VectorXd &u) {
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

Eigen::MatrixXd extended_kalman_filter::jacobF(const Eigen::VectorXd &x, const Eigen::VectorXd &u) {
    Eigen::MatrixXd jF_ = Eigen::Matrix4d::Identity();
//    float yaw = x(2);
//    float v = u(0);
//    jF_(0,2) = -DT * v * std::sin(yaw);
//    jF_(0,3) = DT * std::cos(yaw);
//    jF_(1,2) = DT * v * std::cos(yaw);
//    jF_(1,3) = DT * std::sin(yaw);
    return jF_;
}

Eigen::MatrixXd extended_kalman_filter::jacobH() const {
    //# Jacobian of Observation Model
    Eigen::Matrix4d jH;
    jH.setIdentity();
    return jH;
}

Eigen::VectorXd extended_kalman_filter::observation_model(const Eigen::VectorXd &x) {
    Eigen::Matrix4d H_;
    H_.setIdentity();
    return H_ * x;
}

void extended_kalman_filter::update(const Eigen::VectorXd &z, const Eigen::VectorXd &u) {
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

void extended_kalman_filter::update_cmd(Twist *cmd)     {
    u_.resize(CONTROL_DIM);
    u_ <<   cmd->linear.x,
            cmd->linear.y,
            cmd->linear.z,
            cmd->angular.z;
}

void extended_kalman_filter::update(const std::vector<double>& obs, std::vector<double>& result)
{
    Eigen::VectorXd z(obs.size());
    for (int i = 0; i < obs.size(); ++i) {
        z(i) = obs[i];
    }
    update(z, u_);
    for (int j = 0; j < result.size(); ++j) {
        result[j] = xEst_(j);
    }
}
