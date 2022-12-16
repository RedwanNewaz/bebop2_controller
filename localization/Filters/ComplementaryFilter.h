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
    std::vector<double> update(const std::vector<double>& obs)
    {
        for (int i = 0; i < obs.size(); ++i) {
            X_[i] = alpha_ * X_[i] + (1 - alpha_) * obs[i];
        }
        return X_;
    }
private:
    double alpha_;
    std::vector<double> X_;
};


#endif //BEBOP2_CONTROLLER_COMPLEMENTARYFILTER_H
