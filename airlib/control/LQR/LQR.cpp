//
// Created by redwan on 12/15/22.
//

#include "LQR.h"


namespace bebop2 {
    bool LQR::solveDARE(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B, const Eigen::MatrixXd &Q,
                        const Eigen::MatrixXd &R, Eigen::MatrixXd *X) const {
        // borrowed from https://github.com/wcaarls/grl/blob/master/addons/lqr/src/lqr.cpp

        Eigen::MatrixXd At = A.transpose(), Bt = B.transpose();
        *X = Q;

        double d = EPS;
        for (size_t ii=0; ii < MAX_ITERATION && d >= EPS; ++ii)
        {
            Eigen::MatrixXd Xp = *X;

            *X = Q + At*(*X)*A - At*(*X)*B*(Bt*(*X)*B+R).inverse()*Bt*(*X)*A;
            d = (*X - Xp).array().abs().sum();
        }

        return d >= EPS;
    }

    bool LQR::solve() {
        Eigen::MatrixXd X;

        if (!solveDARE(A, B, Q, R, &X))
        {
            WARNING("Could not solve DARE: error ");
            return false;
        }
        DEBUG(X);
        // Compute feedback gain matrix
        Eigen::MatrixXd K = (B.transpose()* X * B + R).inverse()*(B.transpose() * X * A);

//        Eigen::MatrixXd U = -K * ERROR;
        return true;
    }

    LQR::LQR() {
        dt_ = 0.03;
        A = Eigen::Matrix4d::Identity();
        B = dt_ * Eigen::Vector4d::Identity();
        Q = Eigen::Matrix4d::Identity();
        R = Eigen::Matrix4d::Identity();
    }
} // bebop2