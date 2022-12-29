//
// Created by Redwan Newaz on 12/29/22.
//
#include <memory>
#include "filters_common/helper.h"
#include "extended_kalman_filter.h"


int main(int argc, char* argv[])
{



    const int STATE_DIM = 4;

    const double DT = 0.03;

    std::vector<double>x0{0,0,1,0};

    /*
     * Sigmas - just an estimate, usually comes from uncertainty of sensor, but
     * if you used fused data from multiple sensors, it's difficult to find
     * these uncertainties directly.
     */
    std::vector<double> sigma_pos{0.015, 0.015, 0.015, 0.01}; // GPS measurement uncertainty [x [m], y [m], z [m], theta [rad]]

    auto ekf = std::make_shared<extended_kalman_filter>(x0, sigma_pos, DT, STATE_DIM);

    Simulator sim(STATE_DIM, ekf);
    sim.run();


    return 0;
}