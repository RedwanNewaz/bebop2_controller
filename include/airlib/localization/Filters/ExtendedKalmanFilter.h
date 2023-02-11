//
// Created by Redwan Newaz on 12/29/22.
//

#ifndef PARTICLEFILTER_EXTENDED_KALMAN_FILTER_H
#define PARTICLEFILTER_EXTENDED_KALMAN_FILTER_H

#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include "geometry_msgs/Twist.h"
#include <airlib/localization/Filters/FilterBase.h>

namespace bebop2{
    /**
    *   @brief The extended Kalman Filter uses an algorithm to estimate the state of the system like velocity and position of the drone
    *   by using past and possible noisy observations and current and possibly noisy measurements of that system.
    *   
    *   It estimates the next state of the drone and corrects the estimated state with actual measurements.
    */
    class ExtendedKalmanFilter: public FilterBase{
    public:
        ExtendedKalmanFilter(const std::vector<double>& sigma_pos, double dt, int state_dim);
        void init(const std::vector<double>& X0) override;
        /// @brief Populates it with the estimated state of the system.
        /// @param state Vector of tyoe double indicating the state.
        void operator()(std::vector<double>& state);
        /// @brief Updates the command velocity of the drone by changing the linear x,y,z and angular z values.
        /// @param cmd A pointer that points to the cmd_vel topic. 
        void update_cmd(const geometry_msgs::Twist::ConstPtr& cmd);
        /// @brief In this method, the input obs is converted from a std::vector to an Eigen vector z and then passed to the internal_update() method. 
        /// After the internal_update method is called, the updated state estimate stored in xEst_ is copied into the output result vector.
        /// @param obs A vector of measured values 
        /// @param result A vector to store the update state estimate
        void update(const std::vector<double>& obs, std::vector<double>& result) override;

;


    private:
        Eigen::VectorXd xEst_;
        Eigen::MatrixXd PEst_;
        Eigen::MatrixXd Q_;
        Eigen::MatrixXd R_;
        Eigen::VectorXd u_;

    protected:
        // x_{t+1} = F@x_{t}+B@u_t
        Eigen::VectorXd motion_model(const Eigen::VectorXd& x, const Eigen::VectorXd& u);
        /// @brief Calculates the Jacobian matrix of the motion model
        /// @param x State variable
        /// @param u 
        /// @return Returns the Jacobian matrix
        Eigen::MatrixXd jacobF(const Eigen::VectorXd& x, const Eigen::VectorXd& u);
        /// @brief Calculates the predicted measurement values
        /// @param x State variable
        /// @return Returns the predicted measurement values
        Eigen::VectorXd observation_model(const Eigen::VectorXd& x);
        /// @brief Checks the linearity between the state and the measured variables.
        /// @return Jacobian matrix of the observation model.  
        Eigen::MatrixXd jacobH() const;
        /** @brief The internal update step is used to update the linearized measurement model, which is used to calculate the Kalman gain. 
        * The internal transformation step involves updating the Jacobian matrix that linearizes the measurement model around the current state estimate, and the measurement noise covariance.
        * @param z A vector of measured values. This is the measurement data that is used to update the state estimate.
        * @param u  A vector of control inputs. This is the control data that is used in the motion model to predict the state estimate.
        */ 
        void internal_update(const Eigen::VectorXd& z, const Eigen::VectorXd& u);
        double DT, STATE_DIM, CONTROL_DIM;


    };
}


#endif //PARTICLEFILTER_EXTENDED_KALMAN_FILTER_H
