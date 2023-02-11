//
// Created by redwan on 1/14/23.
//

#ifndef BEBOP2_CONTROLLER_FILTERBASE_H
#define BEBOP2_CONTROLLER_FILTERBASE_H
#include <iostream>
#include <memory>
#include <vector>

class FilterBase;

typedef std::shared_ptr<FilterBase> FilterPtr;
/// @brief The FilterBase class enables various other classes to share the FilterPtr. This class allows other classes like ComplementaryFilter.h,
/// ExtendedKalmanFilter.h,Particle.h to use methods like init(), update() to store the estimated state and update the state.
class FilterBase: std::enable_shared_from_this<FilterBase>
{
public:
    /// @brief  Constructor
    FilterBase()
    {

    }
    /// @brief Deconstructor
    virtual ~FilterBase()
    {

    }
    /// @brief systematic way to share FilterPtr to control class 
    /// @return  FilterPtr 
    FilterPtr getPtr()
    {
        return shared_from_this();
    }

    /// @brief Makes the current estimate of the state of the system start out being the same as the true state.
    /// @param X0 A vector of initial values for the state estimate. 
    virtual void init(const std::vector<double>& X0) = 0;
    /// @brief Updated state is calculated and stored.
    /// @param obs A vector of measured values
    /// @param result A vector to store the update state estimate
    virtual void update(const std::vector<double>& obs, std::vector<double>& result) = 0;
protected:
    /// @brief Estimated state of the system.
    std::vector<double> X_;

};

#endif //BEBOP2_CONTROLLER_FILTERBASE_H
