//
// Created by redwan on 12/15/22.
//

#ifndef BEBOP2_CONTROLLER_STATEOBSERVER_H
#define BEBOP2_CONTROLLER_STATEOBSERVER_H
#include <vector>
#include <memory>
namespace bebop2 {
    template<class Sensor, class Filter>
    class StateObserver {
    public:
        explicit StateObserver(std::shared_ptr<Filter>& filter);
        std::vector<double> get_state();




    private:
        void update_state();
    protected:
        /* Quadrotor state
         * x, y, z, yaw
         */
        std::vector<double> m_state;
        std::shared_ptr<Filter> m_filter;
        Sensor m_sensor;
        bool m_initialized;

    };




} // bebop2
#endif //BEBOP2_CONTROLLER_STATEOBSERVER_H
