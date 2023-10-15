//
// Created by airlab on 10/11/23.
//

#ifndef BEBOP2_CONTROLLER_COMPLEMENTARYFILTERWITHCOV_H
#define BEBOP2_CONTROLLER_COMPLEMENTARYFILTERWITHCOV_H
#include "ComplementaryFilter.h"
#include <Eigen/Dense>
#include <chrono>
#include <algorithm>

class ComplementaryFilterWithCov: public FilterBase
{
    using TIMEPOINT = std::chrono::time_point<std::chrono::high_resolution_clock>;
public:
    ComplementaryFilterWithCov(double alpha)
    {
        m_mean = std::make_unique<ComplementaryFilter>(alpha);
        m_std = std::make_unique<ComplementaryFilter>(1.0-alpha);

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
//            std[i] = std::clamp(sqrt(var), -1.0, 1.0);
        }

        m_std->update(std, std_);


        // during initialization phase velocity is zero
        if(last_state_.empty())
        {
            std::copy(result.begin(), result.end(), std::back_inserter(last_state_));
            last_update_ = std::chrono::high_resolution_clock::now();
        }

        // keep track of previous state to compute velocity
        std::vector<double>velocity(result.size());
        auto current = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed_seconds = current - last_update_;
        double dt = elapsed_seconds.count();

        for (size_t i = 0; i < velocity.size(); ++i) {
            velocity[i] = (result[i] - last_state_[i]) / dt;
//            velocity[i] = std::clamp(velocity[i], -1.0, 1.0);
        }

        last_update_ = std::chrono::high_resolution_clock::now();
        std::copy(result.begin(), result.end(), last_state_.begin());

        // append velocity information at the end of result vector
        // such that (x, y, z, theta, x_dot, y_dot, z_dot, theta_dot)
        std::copy(velocity.begin(), velocity.end(), std::back_inserter(result));

    }

    std::vector<double> getCovVec() const override
    {
        if(std_.empty())
            return {};

        Eigen::VectorXd stdVec(6);
         stdVec << std_[0], std_[1], std_[2], 0, 0, std_[2];
//        stdVec << std_[0], std_[1], std_[2], 0, 0, 0; // temporary ignoring yaw
        Eigen::Matrix<double, 6, 6> cov = stdVec * stdVec.transpose();
        std::vector<double> result;
        for (int i = 0; i < 6; ++i) {
            for (int j = 0; j < 6; ++j) {
                double val = cov(i, j);
//                if( i == 5 || j == 5)
//                    val = std::clamp(val, -0.0872665, 0.0872665);// -5deg to 5 deg
                result.push_back(val);
            }
        }

//        // make sure each covariance value remains between -1 to 1


//        std::transform(result.begin(), result.end(), result.begin(),
//                       [](double val){return std::isnan(val) ? 0.0 : val;});
//
//        std::transform(result.begin(), result.end(), result.begin(),
//                       [](double val){return std::clamp(val, -1.50, 1.50);});

        return result;
    }
private:
    std::unique_ptr<ComplementaryFilter> m_mean;
    std::unique_ptr<ComplementaryFilter> m_std;
    std::vector<double> last_state_, std_;
    ///@brief keep track of time
    TIMEPOINT last_update_;


};

#endif //BEBOP2_CONTROLLER_COMPLEMENTARYFILTERWITHCOV_H
