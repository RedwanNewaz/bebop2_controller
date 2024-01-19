//
// Created by redwan on 10/15/23.
//

#include "ConstantVelocities.h"

namespace airlib {
    ConstantVelocities::ConstantVelocities(double dt):dt_(dt) {

        A_.resize(8, 8);
        B_.resize(8, 4);
        C_.resize(4, 8);

        // Define system dynamics (state transition matrix)
        A_ <<    1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

        // Define input matrix
        B_ <<   dt, 0.0, 0.0, 0.0,
                0.0, dt, 0.0, 0.0,
                0.0, 0.0, dt, 0.0,
                0.0, 0.0, 0.0, dt,
                1.0, 0.0, 0.0, 0.0,
                0.0, 1.0, 0.0, 0.0,
                0.0, 0.0, 1.0, 0.0,
                0.0, 0.0, 0.0, 1.0;

        // Define observation matrix
        C_ <<   1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0;

    }

    Eigen::MatrixXd ConstantVelocities::getA() const {
        return A_;
    }

    Eigen::MatrixXd ConstantVelocities::getB() const {
        return B_;
    }

    Eigen::MatrixXd ConstantVelocities::predict(const Eigen::MatrixXd& x, const Eigen::MatrixXd& u) const {
        return A_ * x + B_ * u;
    }

    Eigen::MatrixXd ConstantVelocities::observe(const Eigen::MatrixXd &x) const {
        return C_ * x;
    }
} // airlib