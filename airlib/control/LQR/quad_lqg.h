//
// Created by airlab on 10/13/23.
//

#ifndef BEBOP2_CONTROLLER_QUAD_LQG_H
#define BEBOP2_CONTROLLER_QUAD_LQG_H

#include <boost/array.hpp>
#include "LQRController.h"
#include "airlib/control/ControllerBase.h"

namespace controller {

    class quad_lqg: public ControllerBase{
    public:
        quad_lqg(const std::vector<double> &gains, double dt);

        void compute_control(const std::vector<double> &X, const std::vector<double> &setPoints,
                             std::vector<double> &control) override;

        void setObsNoise(const std::array<double, 36>& obs) override;

    private:
        double dt_;
        std::vector<double> gains_;
        std::vector<double> prev_x_;
        LQRController _quadController[NUM_CONTROLLER];

    protected:
        std::vector<double> estimate_velocity(const std::vector<double> &X);

    };

} // controller

#endif //BEBOP2_CONTROLLER_QUAD_LQG_H
