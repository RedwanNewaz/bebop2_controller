//
// Created by airlab on 10/13/23.
//

#include "quad_lqg.h"

namespace controller {
    quad_lqg::quad_lqg(const std::vector<double> &gains, double dt): gains_(gains), dt_(dt) {

        std::vector<double> obsNoise{
                0.0003, 0.0, 0.0, 0.0,
                0.0, 0.0001, 0.0, 0.0,
                0.0, 0.0, 0.0001, 0.0,
                0.0, 0.0, 0.0, 0.0001
        };

        for (int i = 0; i < NUM_CONTROLLER; ++i) {
            _quadController[i].set(dt, 0.1, gains, obsNoise);
            vel_.push_back(0.0);
        }


    }

    void quad_lqg::compute_control(const std::vector<double> &X, const std::vector<double> &setPoints,
                                   std::vector<double> &control) {


        std::vector<double> rawControl(NUM_CONTROLLER);
        auto velocities = estimate_velocity(X);
        for (int i = 0; i < NUM_CONTROLLER; ++i) {
            // update position
            _quadController[i].updateState(X[i], i);
            // update velocity
//            _quadController[i].updateState(velocities[i], i + 4);
//            _quadController[i].updateState(0.0, i + 4);
            _quadController[i].updateGoal(velocities[i], i + 4);

            //update goal
            _quadController[i].updateGoal(setPoints[i], i);
            rawControl[i] =(_quadController[i].isTerminated())? 0.0 : _quadController[i].getControl()(i, 0);

            rawControl[i] = std::clamp(rawControl[i], -1.0, 1.0);
            bool normalized = (i == NUM_CONTROLLER - 1);
            if(normalized)
            {
                rawControl[i] = fmod((rawControl[i] + M_PI) , (2 * M_PI)) - M_PI ; //# Normalize between -π and π
            }

            vel_[i] = rawControl[i];

        }
        // calculate orientation
        double theta = -X[3];
        double c = cos(theta);
        double s = sin(theta);
        Eigen::Matrix3d q;
        q.setIdentity();
        q(0, 0) = c;
        q(0, 1) = -s;
        q(1, 0) = s;
        q(1, 1) = c;

        // fix control axis
        Eigen::Vector3d p(rawControl[0], rawControl[1], rawControl[2]);
        Eigen::Vector3d u = q * p;

        // update final control
        control.push_back(u(0));
        control.push_back(u(1));
        control.push_back(u(2));
        control.push_back(rawControl[3]);

    }

  

    std::vector<double> quad_lqg::estimate_velocity(const std::vector<double> &X) {
        if(prev_x_.empty())
            std::copy(X.begin(), X.end(), std::back_inserter(prev_x_));
        std::vector<double> velocity(X.size());
        for (int i = 0; i < X.size(); ++i) {
            velocity[i] = (X[i] - prev_x_[i]) / dt_;
//            velocity[i] = std::clamp(velocity[i], -1.0, 1.0);
        }
        std::copy(X.begin(), X.end(), prev_x_.begin());
        return velocity;
    }

    void quad_lqg::setObsNoise(const std::array<double, 36> &noise) {
        std::vector<double> obsNoise;
        for (int k = 0; k < noise.size(); ++k) {
            int row = k / 4;
            int col = k % 4;
            // x, y, z, r, p, y
            if(row == 3 || row == 4 || col == 3 || col == 4)
                continue;
            obsNoise.push_back(noise[k]);
        }
        
        for (size_t i = 0; i < NUM_CONTROLLER; ++i) {
            _quadController[i].set(dt_, 0.1, gains_, obsNoise);
        }
    }
} // controller