//
// Created by Abdullah Al Redwan Newaz on 10/13/23.
//

#ifndef LQR_LQR_H
#define LQR_LQR_H


// LQRController.h
#pragma once

#include <Eigen/Dense>

using namespace Eigen;

class LQRController {
private:
    MatrixXd A, B, Q, R, P, K;

public:
    LQRController(double dt);

    MatrixXd computeControlInput(const MatrixXd &x, const MatrixXd &xg);

    MatrixXd predict(const MatrixXd &x, const MatrixXd& u);

protected:
    MatrixXd solveContinuousARE();

    MatrixXd calculateLQRControllerGain();

};



#endif //LQR_LQR_H
