//
// Created by redwan on 12/15/22.
//

#ifndef BEBOP2_CONTROLLER_STATEOBSERVER_H
#define BEBOP2_CONTROLLER_STATEOBSERVER_H
#include <iostream>
#include <vector>
#include <memory>
#include "filters.h"
#include "sensors.h"


namespace bebop2 {
    class StateObserver;
    typedef std::shared_ptr<StateObserver> StateObserverPtr;
    /**
    *   @brief The StateObserver class Observes and updates the state of the drone
    * 
    *    and sends those messages to different modules of the program such as controllers, sensors etc so that they can control the drone and perform tasks.
    */
    
    class StateObserver {
    public:
        /// @brief Initializes the filter and sensor parameter and sets m_initliazed to False.
        /// @param filter A barrier to refine the noise.
        /// @param sensor 
        /// 
        explicit StateObserver(FilterPtr filter, SensorPtr sensor);

        /// @brief It updates and copies the state of the drone and adds the copied state into the parameter result.
        /// @param result 
        void operator()(std::vector<double>& result);
    private:
        //
        // Accesses the internal state vector
        // Publishes robot transformation
        void update_state();
    protected:
        /* Quadrotor state
         * x, y, z, yaw
        */
        /** @brief This attribute is an array of values and the elements in the vector gives the state of the robot. The first three
        * attributes are vectors that gives the linear state of the robot and yaw gives the rotation of the robot about these vectors
        */
        /**   @note The elements in the array are commonly represented by:-
        *           - x ---> Position of the drone in x-axis.
        *           - y ---> Position of the drone in x-axis.
        *           - z ---> Position of the drone in z-axis.
        *           - w ---> yaw (Rotation about z-axis).
        */  
        std::vector<double> m_state;
        /// @brief Shared pointer that provides the address to the FilterBase.h class
        FilterPtr m_filter;
        /// @brief Shared pointer that provides the addreess to the SensorBase.h class
        SensorPtr m_sensor;
        bool m_initialized;

    };

} // bebop2
#endif //BEBOP2_CONTROLLER_STATEOBSERVER_H
