//
// Created by Redwan Newaz on 12/27/22.
//

#ifndef PARTICLEFILTER_HELPER_H
#define PARTICLEFILTER_HELPER_H
#include <iostream>
#include <random>
#include <vector>
#include <math.h>
#include <iterator>
#include <fstream>
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;


struct Twist{
    struct LinearVel{
        double x, y, z;
    }linear;
    struct AngularVel{
        double x, y, z;
    }angular;
    Twist(double x, double y, double z, double yaw)
    {
        linear.x = x;
        linear.y = y;
        linear.z = z;
        angular.z = yaw;
    }

    friend std::ostream &operator<<(std::ostream &os, const Twist &twist) {
        os << "linear: x = " << twist.linear.x  << " linear: y = " << twist.linear.y  << " linear: z = " << twist.linear.z  << " angular: z = " << twist.angular.z;
        return os;
    }

};

class StateSpace{
public:
    StateSpace()
    {
        states_.resize(STATE_DIM);
        std::fill(states_.begin(), states_.end(), 0.0);
        states_[2] = 1.0;
    }

    void operator()(std::vector<double>& state)
    {
//        state.clear();
        std::copy(states_.begin(), states_.end(), state.begin());

        std::normal_distribution disturbance(0.0, sqrt(NOISE));

        for (int i = 0; i < state.size(); ++i) {

            state[i] += disturbance(rd_);
        }

    }

    void add_control(const Twist* msg)
    {
        states_[0] += DT * msg->linear.x;
        states_[1] -= DT * msg->linear.y;
        states_[2] += DT * msg->linear.z;
        states_[3] += DT * msg->angular.z;
    }

private:
    std::vector<double> states_;
    const size_t STATE_DIM = 4;
    const double DT = 0.03;
    const double NOISE = 0.015;
    std::random_device rd_;
};





template <typename T>
class Simulator{
public:
    Simulator(const int state_dim, std::shared_ptr<T>& filter): filter_(std::move(filter)), STATE_DIM(state_dim)
    {

    }

    void run()
    {
        const int MAX_ITER = 4 * 690;
        size_t SIM_TIME = 0;
        StateSpace quadState;
        std::vector<double> xf(MAX_ITER), xx(MAX_ITER), yf(MAX_ITER), yy(MAX_ITER);

        do{
            Twist *cmd;

            if(SIM_TIME < 690)
            {
                cmd = new Twist(0.1, 0, 0, 0);
            }
            else if(SIM_TIME < 2 * 690)
            {
                cmd = new Twist(0, -0.1, 0, 0);
            }
            else if(SIM_TIME < 3 * 690)
            {
                cmd = new Twist(-0.1, 0, 0, 0);
            }
            else
            {
                cmd = new Twist(0, 0.1, 0, 0);
            }

            quadState.add_control(cmd);

            std::vector<double> state_obs(STATE_DIM);
            quadState(state_obs);
            filter_->update_cmd(cmd);

            std::vector<double> state_filtered(STATE_DIM, 0.0);
            filter_->update(state_obs, state_filtered);

            filter_->operator()(state_filtered);



            xx[SIM_TIME] = state_obs[0];
            yy[SIM_TIME] = state_obs[1];

            xf[SIM_TIME] = state_filtered[0];
            yf[SIM_TIME] = state_filtered[1];

            std::cout << "Simulation Step remaining = " << MAX_ITER - SIM_TIME << std::endl;

            delete cmd;

        }while (++SIM_TIME <= MAX_ITER);

        plt::scatter(xx, yy);
        plt::scatter(xf, yf);
        plt::show();
    }

private:
    std::shared_ptr<T> filter_;
    const int STATE_DIM;
};




#endif //PARTICLEFILTER_HELPER_H
