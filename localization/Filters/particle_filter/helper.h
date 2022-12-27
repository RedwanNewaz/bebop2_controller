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







#endif //PARTICLEFILTER_HELPER_H
