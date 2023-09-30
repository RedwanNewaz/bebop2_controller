//
// Created by redwan on 9/30/23.
//

#ifndef BEBOP2_CONTROLLER_QUAD_PIDS_H
#define BEBOP2_CONTROLLER_QUAD_PIDS_H

#include "../ControllerBase.h"
#include "airlib/control/pid/PID.h"
#include <Eigen/Dense>
#include <vector>

namespace controller{
    /// @brief Responsible to find the error values every second whenever the position of the quadrators of the drone is updated.
    class quad_pids: public ControllerBase{
    public:
        /** @brief The QuadControllerPID constructor initializes a vector of type double that is responsible for populating the PID Gains, maximum and minimum threshold values.
        *   @param mGetState This parameter givs the current state of the robot.
        *   @param nh Used to setup publishers, subscribers and deals with various messages.
        */
        quad_pids(const std::vector<double> &gains, double dt);

        /// @brief Similar to the calculate() method in PID.h class, the compute_control method also calculates the error, PID Gain constants and return the error value when the drone is hovering over a position for the four vector values.
        /// @param X is the current state of the drone.
        /// @param setPoints is the desired goal position of the drone.
        /// @param control is the actively control position.
        void compute_control(const std::vector<double> &X, const std::vector<double> &setPoints,
                             std::vector<double> &control) override;

    private:
        /// @brief Sets all the proportional, integral, and derivative constants along with the maximum and minimum output values.
        /// @param gains vector of size 4 and of type double that stores the PID controller parameters.
        /** @note PID Gains identifiers.
        *          - Kp = Proportionality Constant
        *          - Kd = Derivative Constant
        *          - Ki = Intergration Constant
        */
        bebop2::PID _quadController[NUM_CONTROLLER];
    };
}
#endif //BEBOP2_CONTROLLER_QUAD_PIDS_H
