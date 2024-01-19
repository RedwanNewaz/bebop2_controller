//
// Created by redwan on 12/15/22.
//

#include "StateObserver.h"


namespace bebop2 {


    StateObserver::StateObserver(FilterPtr filter, SensorPtr sensor):
    m_filter(filter), m_sensor(sensor) {
        m_initialized = false;
    }


    void StateObserver::getState(std::promise<std::vector<double>>& promise ) {

        while (m_state.empty())
        {
            update_state();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        std::vector<double> result;
        std::copy(m_state.begin(), m_state.end(),std::back_inserter(result));
        m_state.clear();
        promise.set_value(result);
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
            m_cov = m_filter->getCovVec();

//            std::cout << m_state[0] << ", " << m_state[1] << ", " << m_state[2] << ", " << m_state[3] << std::endl;
        }

        if(!m_state.empty())
            m_sensor->publish_tf(m_state);

    }

    void StateObserver::getStateWithCov(std::promise<std::vector<double>> &promiseState,
                                        std::promise<std::vector<double>> &promiseCov) {

        while (m_state.empty())
        {
            update_state();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        std::vector<double> result;
        std::copy(m_state.begin(), m_state.end(),std::back_inserter(result));
        m_state.clear();
        promiseState.set_value(result);
        promiseCov.set_value(m_cov);

    }


} // bebop2