//
// Created by Redwan Newaz on 12/29/22.
//

#ifndef PARTICLEFILTER_EXTENDED_KALMAN_FILTER_H
#define PARTICLEFILTER_EXTENDED_KALMAN_FILTER_H

#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include "../include/helper.h"


class extended_kalman_filter {
public:
    extended_kalman_filter(const std::vector<double> &state, const std::vector<double>& sigma_pos, double dt, int state_dim);
    void operator()(std::vector<double>& state);
    void update_cmd(Twist *cmd);

    // interface
    void update(const std::vector<double>& obs, std::vector<double>& result);


private:
    Eigen::VectorXd xEst_;
    Eigen::MatrixXd PEst_;
    Eigen::MatrixXd Q_;
    Eigen::MatrixXd R_;
    Eigen::VectorXd u_;
    
protected:
    // x_{t+1} = F@x_{t}+B@u_t
    Eigen::VectorXd motion_model(const Eigen::VectorXd& x, const Eigen::VectorXd& u);
    Eigen::MatrixXd jacobF(const Eigen::VectorXd& x, const Eigen::VectorXd& u);
    Eigen::VectorXd observation_model(const Eigen::VectorXd& x);
    Eigen::MatrixXd jacobH() const;
    void update(const Eigen::VectorXd& z, const Eigen::VectorXd& u);
    double DT, STATE_DIM, CONTROL_DIM;


};


#endif //PARTICLEFILTER_EXTENDED_KALMAN_FILTER_H
