//
// Created by redwan on 12/15/22.
//

#ifndef BEBOP2_CONTROLLER_COMPLEMENTARYFILTER_H
#define BEBOP2_CONTROLLER_COMPLEMENTARYFILTER_H
#include <vector>

class ComplementaryFilter{
public:
    ComplementaryFilter(double alpha):alpha_(alpha)
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
            X_[i] = alpha_ * X_[i] + (1 - alpha_) * obs[i];
        }
        result.clear();
        std::copy(X_.begin(), X_.end(),std::back_inserter(result));
    }
private:
    double alpha_;
    std::vector<double> X_;
};


#endif //BEBOP2_CONTROLLER_COMPLEMENTARYFILTER_H
