//
// Created by Abdullah Al Redwan Newaz on 10/13/23.
//

#include "LQR.h"
#include <iostream>
#include <algorithm>

using namespace std;

LQRController::LQRController(double dt) {
    A.resize(8, 8);
    B.resize(8, 4);
    Q.resize(8, 8);
    R.resize(4, 4);
    P.resize(8, 8);
    K.resize(4, 8);


    // Set system matrices
    // ... (set A and B matrices)

    // Define system dynamics (example values)
    A <<    1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    B <<    dt, 0.0, 0.0, 0.0,
            0.0, dt, 0.0, 0.0,
            0.0, 0.0, dt, 0.0,
            0.0, 0.0, 0.0, dt,
            1.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 1.0;

    // Set cost matrices
    // Define cost matrices (example values)
    MatrixXd Gain(8, 8);
    Gain << 0.075, 0.0, 0.0, 0.0, -0.001, 0.0, 0.0, 0.0,
            0.0, 0.075, 0.0, 0.0, 0.0, -0.00075, 0.0, 0.0,
            0.0, 0.0, 0.085, 0.0, 0.0, 0.0, -0.001, 0.0,
            0.0, 0.0, 0.0, 0.08, 0.0, 0.0, 0.0, -0.002,
            0.0, 0.0, 0.0, 0.0, -0.001, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, -0.00075, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.001, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.002;


    // ... (set Q and R matrices)
    Q << Gain.completeOrthogonalDecomposition().pseudoInverse();

    R <<    0.0003, 0.0, 0.0, 0.0,
            0.0, 0.0001, 0.0, 0.0,
            0.0, 0.0, 0.0001, 0.0,
            0.0, 0.0, 0.0, 0.0001;

    solveContinuousARE();
    K = calculateLQRControllerGain();
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

MatrixXd LQRController::computeControlInput(const MatrixXd &x, const MatrixXd &xg) {
    // Compute control input based on current state and goal state
    MatrixXd error = xg - x;
    MatrixXd u = -K * error;
    return u;
}

MatrixXd LQRController::predict(const MatrixXd &x, const MatrixXd &u) {
    return  A * x + B * u;
}

