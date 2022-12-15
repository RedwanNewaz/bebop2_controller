//
// Created by redwan on 12/15/22.
//

#ifndef BEBOP2_CONTROLLER_LQR_H
#define BEBOP2_CONTROLLER_LQR_H
#include <iostream>
#include <Eigen/Dense>
#define EPS 0.001
#define MAX_ITERATION 1000
#define WARNING(x) std::cerr << x << std::endl
#define DEBUG(x) std::cout << x << std::endl

namespace bebop2 {

    class LQR {
    public:
        LQR();

    private:
        bool solve();
        bool solveDARE(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B, const Eigen::MatrixXd &Q, const Eigen::MatrixXd &R, Eigen::MatrixXd *X) const;
        Eigen::MatrixXd A, B, Q, R;
        double dt_;
    };

} // bebop2

#endif //BEBOP2_CONTROLLER_LQR_H
