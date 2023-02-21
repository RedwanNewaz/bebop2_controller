//
// Created by redwan on 12/15/22.
//

#ifndef BEBOP2_CONTROLLER_COMPLEMENTARYFILTER_H
#define BEBOP2_CONTROLLER_COMPLEMENTARYFILTER_H
#include <vector>
#include "FilterBase.h"


/// @brief The ComplementaryFilter.h class implements the complmentary filter where it initializes the state and updates it using the filter's alpha paramater in update() method.
class ComplementaryFilter: public FilterBase{
public:
    ComplementaryFilter(double alpha): m_alpha(alpha)
    {

    }

    void init(const std::vector<double>& X0)
    {
        X_.clear();
        std::copy(X0.begin(), X0.end(), std::back_inserter(X_));
    }
    void update(const std::vector<double>& obs, std::vector<double>& result)
    {

        for (int i = 0; i < obs.size(); ++i) {
            X_[i] = m_alpha * X_[i] + (1 - m_alpha) * obs[i];
        }
        result.clear();
        std::copy(X_.begin(), X_.end(),std::back_inserter(result));
    }
private:
    double m_alpha;
};


#endif //BEBOP2_CONTROLLER_COMPLEMENTARYFILTER_H
