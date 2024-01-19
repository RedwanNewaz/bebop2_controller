//
// Created by redwan on 10/15/23.
//

#ifndef BEBOP2_CONTROLLER_CONSTANTVELOCITIES_H
#define BEBOP2_CONTROLLER_CONSTANTVELOCITIES_H
#include <Eigen/Dense>
namespace airlib {
    /**
     * @brief linear kinematics model followed by x_dot = Ax + Bu
     * use Newton's equation of motion to predict next state
     * state space contains (x, y, theta, yaw, x_dot, y_dot, theta_dot, yaw_dot)
     */
    class ConstantVelocities {
    public:
        ConstantVelocities(double dt);

        ///@brief get state transition matrix
        Eigen::MatrixXd getA() const;
        ///@brief get control input matrix
        Eigen::MatrixXd getB() const;
        /**
         * predict next state
         * @param x : current state 8x1 vector
         * @param u : control input 4x1 vector
         * @return state at t+1 timestamp
         */
        Eigen::MatrixXd predict(const Eigen::MatrixXd& x, const Eigen::MatrixXd& u) const;
        /// @brief sensor observation y = cx
        Eigen::MatrixXd observe(const Eigen::MatrixXd& x) const;
    private:
        Eigen::MatrixXd A_, B_, C_;
        double dt_;

    };

} // airlib

#endif //BEBOP2_CONTROLLER_CONSTANTVELOCITIES_H
