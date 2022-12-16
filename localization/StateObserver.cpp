//
// Created by redwan on 12/15/22.
//

#include "StateObserver.h"


namespace bebop2 {

    template<class Sensor, class Filter>
    StateObserver<Sensor, Filter>::StateObserver(std::shared_ptr<Filter> &filter, std::shared_ptr<Sensor> &sensor):
    m_filter(filter), m_sensor(sensor) {
        m_initialized = false;
    }

    template<class Sensor, class Filter>
    std::vector<double> StateObserver<Sensor, Filter>::get_state() {

        update_state();
        return m_state;
    }

    template<class Sensor, class Filter>
    void StateObserver<Sensor, Filter>::update_state() {
        while (!m_sensor->empty())
        {
            auto obs = m_sensor->get_observations();
            if(!m_initialized)
            {
                m_filter->init(obs);
                m_initialized = true;
            }
            m_state = m_filter->update(obs);

//            std::cout << m_state[0] << ", " << m_state[1] << ", " << m_state[2] << std::endl;
        }

        if(!m_state.empty())
            m_sensor->publish_tf(m_state);

    }


} // bebop2