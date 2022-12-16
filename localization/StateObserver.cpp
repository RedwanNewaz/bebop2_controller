//
// Created by redwan on 12/15/22.
//

#include "StateObserver.h"

namespace bebop2 {

    template<class Sensor, class Filter>
    StateObserver<Sensor, Filter>::StateObserver(std::shared_ptr<Filter> &filter):m_filter(filter) {
        m_initialized = false;
    }

    template<class Sensor, class Filter>
    std::vector<double> StateObserver<Sensor, Filter>::get_state() {
        update_state();
        return m_state;
    }

    template<class Sensor, class Filter>
    void StateObserver<Sensor, Filter>::update_state() {
        while (!m_sensor.empty())
        {
            auto obs = m_sensor.get_observations();
            if(!m_initialized)
            {
                m_filter->init(obs);
                m_initialized = true;
            }
            m_state = m_filter->update(obs);
        }

    }


} // bebop2