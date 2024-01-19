//
// Created by redwan on 9/30/23.
//

#ifndef BEBOP2_CONTROLLER_CONTROLLERBASE_H
#define BEBOP2_CONTROLLER_CONTROLLERBASE_H
#include <iostream>
#include <memory>
#include <vector>
#include <Eigen/Dense>
#define NUM_CONTROLLER  4

class ControllerBase;

typedef std::shared_ptr<ControllerBase> ControllerPtr;
/// @brief The ControllerBase class enables various other classes to share the FilterPtr. This class allows other classes like ComplementaryFilter.h,
/// ExtendedKalmanFilter.h,Particle.h to use methods like init(), update() to store the estimated state and update the state.
class ControllerBase: std::enable_shared_from_this<ControllerBase>
{
public:
    /// @brief  Constructor
    ControllerBase()
    {

    }
    /// @brief  Deconstructor
    virtual ~ControllerBase()
    {

    }
    /// @brief systematic way to share FilterPtr to control class 
    /// @return  FilterPtr 
    ControllerPtr getPtr()
    {
        return shared_from_this();
    }

    /// @brief set measurement covariance matrix for the LQG controller to reason about uncertainty
    virtual void setObsNoise(const std::array<double, 36>& obs)
    {

    }


    /// @brief Control is calculated from input state and setpoints.
    /// @param X A vector of state values
    /// @param setPoints A vector to represent desire (goal) state
    /// @param control A vector to store the update control inputs
    virtual void compute_control(const std::vector<double> &X, const std::vector<double> &setPoints,
                         std::vector<double> &control)
    {
        // this function should be invoked after a controller computes the control input
        // in inertial frame. This function will transform control to body frame
        assert(control.size() == NUM_CONTROLLER);
        // calculate orientation
        double theta = -X[3];
        double c = cos(theta);
        double s = sin(theta);
        Eigen::Matrix3d q;
        q.setIdentity();
        q(0, 0) = c;
        q(0, 1) = -s;
        q(1, 0) = s;
        q(1, 1) = c;

        // fix control axis
        Eigen::Vector3d p(control[0], control[1], control[2]);
        Eigen::Vector3d u = q * p;

        // update final control
        control[0] = u(0);
        control[1] = u(1);
        control[2] = u(2);
    }
};
#endif //BEBOP2_CONTROLLER_CONTROLLERBASE_H
