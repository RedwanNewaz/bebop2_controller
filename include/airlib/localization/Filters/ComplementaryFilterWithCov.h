//
// Created by airlab on 10/11/23.
//

#ifndef BEBOP2_CONTROLLER_COMPLEMENTARYFILTERWITHCOV_H
#define BEBOP2_CONTROLLER_COMPLEMENTARYFILTERWITHCOV_H
#include "ComplementaryFilter.h"
#include <Eigen/Dense>

class ComplementaryFilterWithCov: public FilterBase
{
public:
    ComplementaryFilterWithCov(double alpha)
    {
        m_mean = std::make_unique<ComplementaryFilter>(alpha);
        m_std = std::make_unique<ComplementaryFilter>(1 - alpha);
    }
    void init(const std::vector<double>& X0)
    {
        m_mean->init(X0);
        std::vector<double> std(X0.size(), 0.0);
        m_std->init(std);
    }

    void update(const std::vector<double>& obs, std::vector<double>& result)
    {
        m_mean->update(obs, result);
        // compute std deviation
        std::vector<double> std(obs.size(), 0.0);
        for (size_t i = 0; i < obs.size(); ++i) {
            double error = (result[i] - obs[i]);
            double var = error * error;
            std[i] = sqrt(var);
        }

        m_std->update(std, std_);
    }

    std::vector<double> getCovVec() const override
    {
        if(std_.empty())
            return {};

        Eigen::VectorXd stdVec(6);
        stdVec << std_[0], std_[1], std_[2], 0, 0, std_[3];
        Eigen::Matrix<double, 6, 6> cov = stdVec * stdVec.transpose();
        std::vector<double> result;
        for (int i = 0; i < 6; ++i) {
            for (int j = 0; j < 6; ++j) {
                double val = cov(i, j);
                result.push_back(val);
            }
        }
        return result;
    }
private:
    std::unique_ptr<ComplementaryFilter> m_mean;
    std::unique_ptr<ComplementaryFilter> m_std;
    std::vector<double> std_;

};

#endif //BEBOP2_CONTROLLER_COMPLEMENTARYFILTERWITHCOV_H
