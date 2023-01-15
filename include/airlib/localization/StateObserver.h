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
    class StateObserver {
    public:
        explicit StateObserver(FilterPtr filter, SensorPtr sensor);
        void operator()(std::vector<double>& result);




    private:
        void update_state();
    protected:
        /* Quadrotor state
         * x, y, z, yaw
         */
        std::vector<double> m_state;
        FilterPtr m_filter;
        SensorPtr m_sensor;
        bool m_initialized;

    };

} // bebop2
#endif //BEBOP2_CONTROLLER_STATEOBSERVER_H
