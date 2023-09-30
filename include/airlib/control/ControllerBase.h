//
// Created by redwan on 9/30/23.
//

#ifndef BEBOP2_CONTROLLER_CONTROLLERBASE_H
#define BEBOP2_CONTROLLER_CONTROLLERBASE_H
#include <iostream>
#include <memory>
#include <vector>
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


    /// @brief Control is calculated from input state and setpoints.
    /// @param X A vector of state values
    /// @param setPoints A vector to represent desire (goal) state
    /// @param control A vector to store the update control inputs
    virtual void compute_control(const std::vector<double> &X, const std::vector<double> &setPoints,
                         std::vector<double> &control) = 0;
};
#endif //BEBOP2_CONTROLLER_CONTROLLERBASE_H
