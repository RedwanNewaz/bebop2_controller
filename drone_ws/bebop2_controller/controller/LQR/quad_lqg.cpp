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
            if(i < NUM_CONTROLLER - 1)
                _quadController[i].set(dt, 0.1, gains, obsNoise);
            else
            {
                _quadController[i].set(dt, 0.0872665, gains, obsNoise);
            }
            vel_.push_back(0.0);
        }


    }

    void quad_lqg::compute_control(const std::vector<double> &X, const std::vector<double> &setPoints,
                                   std::vector<double> &control) {


        control.resize(NUM_CONTROLLER);
        auto velocities = estimate_velocity(X);
        for (int i = 0; i < NUM_CONTROLLER; ++i) {
            // update position
            _quadController[i].updateState(X[i], i);
            // update velocity

            _quadController[i].updateState(velocities[i], i + 4);
            _quadController[i].updateGoal(vel_[i], i + 4);

            //update goal
            _quadController[i].updateGoal(setPoints[i], i);
//            control[i] =(_quadController[i].isTerminated())? 0.0 : _quadController[i].getControl()(i, 0);
            control[i] = _quadController[i].getControl()(i, 0);

            control[i] = std::clamp(control[i], -1.0, 1.0);
            bool normalized = (i == NUM_CONTROLLER - 1);
            if(normalized)
            {
                control[i] = fmod((control[i] + M_PI) , (2 * M_PI)) - M_PI ; //# Normalize between -π and π
            }

            vel_[i] = control[i];

        }
        // transform control to body frame
        ControllerBase::compute_control(X, setPoints, control);

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
            int row = k / 6;
            int col = k % 6;
            // x, y, z, r, p, y
            if(row == 3 || row == 4 || col == 3 || col == 4)
                continue;
            obsNoise.push_back(noise[k]);
        }
//        std::cout << obsNoise.size() << std::endl;
        for (size_t i = 0; i < NUM_CONTROLLER; ++i) {
            _quadController[i].set(dt_, 0.1, gains_, obsNoise);
        }
    }
} // controller