//
// Created by redwan on 12/15/22.
//

#include "airlib/localization/StateObserver.h"


namespace bebop2 {


    StateObserver::StateObserver(FilterPtr filter, SensorPtr sensor):
    m_filter(filter), m_sensor(sensor) {
        m_initialized = false;
    }


    void StateObserver::operator()(std::vector<double>& result) {

        update_state();
        if (result.empty())
            std::copy(m_state.begin(), m_state.end(),std::back_inserter(result));
        else
            std::copy(m_state.begin(), m_state.begin() + result.size(),result.begin());
    }


    void StateObserver::update_state() {
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

//            std::cout << m_state[0] << ", " << m_state[1] << ", " << m_state[2] << ", " << m_state[3] << std::endl;
        }

        if(!m_state.empty())
            m_sensor->publish_tf(m_state);

    }


} // bebop2