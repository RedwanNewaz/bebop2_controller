//
// Created by airlab on 10/13/23.
//


#ifndef LQR_LQR_H
#define LQR_LQR_H


// LQRController.h
#pragma once
#include <memory>
#include <Eigen/Dense>
#include "airlib/model.hpp"
using namespace Eigen;

class LQRController {
private:
    MatrixXd A, B, Q, R, P, K;
    MatrixXd x_, xg_;
    double goalThres_;
    std::unique_ptr<airlib::ConstantVelocities> sys_;

public:
    LQRController();

    MatrixXd getControl() const;

    void set(double dt, double goalThres, const std::vector<double>& gain, const std::vector<double>& obsNoise);

    void updateState(double val, int index);

    void updateGoal(double val, int index);

    MatrixXd predict(const MatrixXd& u);

    bool isTerminated() const;

protected:
    MatrixXd solveContinuousARE();

    MatrixXd calculateLQRControllerGain();

    MatrixXd computeControlInput(const MatrixXd &x, const MatrixXd &xg) const;

};



#endif //LQR_LQR_H