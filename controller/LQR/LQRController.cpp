//
// Created by airlab on 10/13/23.
//

#include "LQRController.h"
#include <iostream>
#include <algorithm>

using namespace std;

LQRController::LQRController() {
    A.resize(8, 8);
    B.resize(8, 4);
    Q.resize(8, 8);
    R.resize(4, 4);
    P.resize(8, 8);
    K.resize(4, 8);
    x_.resize(8, 1);
    xg_.resize(8, 1);

    x_ << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    xg_ << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
//    cout << K << endl;
}

MatrixXd LQRController::solveContinuousARE() {
    // Solve the continuous-time Algebraic Riccati Equation (CARE)
    P = MatrixXd::Zero(A.rows(), A.cols());
    double tolerance = 1e-6;
    int max_iterations = 100;

    for (int i = 0; i < max_iterations; ++i) {
        MatrixXd P_next = A.transpose() * P * A - A.transpose() * P * B * (R + B.transpose() * P * B).inverse() * B.transpose() * P * A + Q;
        if ((P - P_next).norm() < tolerance) {
            P = P_next;
            break;
        }
        P = P_next;
    }

    return P;
}

MatrixXd LQRController::calculateLQRControllerGain() {
    // Calculate the LQR controller gain
    K = (R + B.transpose() * P * B).inverse() * B.transpose() * P * A;
    return K;
}

MatrixXd LQRController::computeControlInput(const MatrixXd &x, const MatrixXd &xg) const{
    // Compute control input based on current state and goal state
    MatrixXd error = xg - x;
    MatrixXd u0 = x_.block<4, 1>(4, 0);
    MatrixXd u = u0 + K * error;
    MatrixXd uDefault(4, 1);
    uDefault.setZero();
    return isTerminated()? uDefault : u;
}

MatrixXd LQRController::predict(const MatrixXd &u) {

    return sys_->predict(x_, u);
}

void LQRController::updateState(double val, int index) {
    x_(index) = val;
}

void LQRController::updateGoal(double val, int index) {
    xg_(index) = val;
}

MatrixXd LQRController::getControl() const {
    return computeControlInput(x_, xg_);
}

bool LQRController::isTerminated() const{
    MatrixXd xPos = x_.block<3, 1>(0, 0);
    MatrixXd gPos = xg_.block<3, 1>(0, 0);
    double dist = (gPos - xPos).norm();
//    printf(" error (%.2lf) \n", dist);
    return (dist < goalThres_);

}

void LQRController::set(double dt, double goalThres, const std::vector<double>& gain, const std::vector<double>& obsNoise ) {

    sys_ = std::make_unique<airlib::ConstantVelocities>(dt);
    A = sys_->getA();
    B = sys_->getB();
    this->goalThres_ = goalThres;

    // Set cost matrices
    // Define cost matrices (example values)
    MatrixXd Gain(8, 8);
    for (size_t i = 0; i < gain.size(); ++i) {
        int row = i / 8;
        int col = i % 8;
        Gain(row, col) = gain[i];
    }

    // ... (set Q and R matrices)
    Q << Gain.completeOrthogonalDecomposition().pseudoInverse();

    for (size_t j = 0; j < obsNoise.size(); ++j) {
        int row = j / 4;
        int col = j % 4;
        R(row, col) = obsNoise[j];
    }
    solveContinuousARE();
    K = calculateLQRControllerGain();
}
