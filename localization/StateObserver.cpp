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
    void StateObserver<Sensor, Filter>::operator()(std::vector<double>& result) {

        update_state();
        result.clear();
        std::copy(m_state.begin(), m_state.end(),std::back_inserter(result));
    }

    template<class Sensor, class Filter>
    void StateObserver<Sensor, Filter>::update_state() {
        while (!m_sensor->empty())
        {
            std::vector<double> obs;
            m_sensor->operator()(obs);
            if(!m_initialized)
            {
                m_filter->init(obs);
                m_initialized = true;
            }
            m_filter->update(obs, m_state);

//            std::cout << m_state[0] << ", " << m_state[1] << ", " << m_state[2] << std::endl;
        }

        if(!m_state.empty())
            m_sensor->publish_tf(m_state);

    }


} // bebop2