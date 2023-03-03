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
    u_.resize(CONTROL_DIM);
    u_ << 0, 0, 0, 0;

//    PEst_ = PEst_ * 666e-3;

//    PEst_(2, 2) = PEst_(2, 2) * 5e-2;


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
    return jF_;
}

Eigen::MatrixXd ExtendedKalmanFilter::jacobH() const {
    //# Jacobian of Observation Model
    Eigen::MatrixXd jH = Eigen::Matrix4d::Identity();

    return jH;
}

Eigen::VectorXd ExtendedKalmanFilter::observation_model(const Eigen::VectorXd &x) {
    Eigen::MatrixXd H_ = Eigen::Matrix4d::Identity();
    return H_ * x;
}

void ExtendedKalmanFilter::internal_update(const Eigen::VectorXd &z, const Eigen::VectorXd &u) {

    Eigen::MatrixXd H_ = Eigen::Matrix4d::Identity();
    Eigen::VectorXd z_pred = H_ * xEst_;
    Eigen::VectorXd y = z - z_pred;
    Eigen::MatrixXd Ht = H_.transpose();
    Eigen::MatrixXd PHt = PEst_ * Ht;
    Eigen::MatrixXd S = H_ * PHt + R_;
    Eigen::MatrixXd Si = S.inverse();
    Eigen::MatrixXd K = PHt * Si;

    //new estimate
    xEst_ = xEst_ + (K * y);
    long x_size = xEst_.size();
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(x_size, x_size);
    PEst_ = (I - K * H_) * PEst_;



}

void ExtendedKalmanFilter::update_cmd(const geometry_msgs::Twist::ConstPtr& cmd)     {

    u_(0) =   cmd->linear.x;
    u_(1) =   cmd->linear.y;
    u_(2) =   cmd->linear.z;
    u_(3) =   cmd->angular.z;

}

void ExtendedKalmanFilter::update(const std::vector<double>& obs, std::vector<double>& result)
{

    Eigen::VectorXd z(obs.size());
    for (int i = 0; i < obs.size(); ++i) {
        z(i) = obs[i];

    }
//    z(3) = 3.1416;



    internal_update(z, u_);
//    std::cout << "[update] finished internal update " << xEst_.transpose() << std::endl ;
////    std::cout << "[update] finished internal update " << xEst_.transpose() ;
//    result.clear();
    if(result.size() != STATE_DIM)
        result.resize(STATE_DIM);

    for (int j = 0; j < STATE_DIM; ++j) {
        result[j] = xEst_(j);
    }
}

void ExtendedKalmanFilter::init(const std::vector<double> &X0) {
    for (int j = 0; j < STATE_DIM; ++j) {
        xEst_(j) = X_[j] = X0[j];
    }
}
