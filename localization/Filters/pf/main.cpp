#include "../include/filters_common/helper.h"
#include "particle_filter.h"
#include "filters_common/matplotlibcpp.h"
#include <memory>

namespace plt = matplotlibcpp;

int main() {

    //Set up parameters here
    int num_particles  = 500;
    const int STATE_DIM = 4;
    const double DT = 0.03;

    std::vector<double>x0{0,0,1,0};

    /*
     * Sigmas - just an estimate, usually comes from uncertainty of sensor, but
     * if you used fused data from multiple sensors, it's difficult to find
     * these uncertainties directly.
     */
    std::vector<double> sigma_pos{0.015, 0.015, 0.015, 0.01}; // GPS measurement uncertainty [x [m], y [m], z [m], theta [rad]]

    TagMap tagMap;
    tagMap.landmark_list.push_back({0, 5, 0, 0.95});
    tagMap.landmark_list.push_back({1, 5, 2.05, 0.95});
    tagMap.landmark_list.push_back({2, 5, 1.13, 0.95});


    auto pf = std::make_shared<ParticleFilter>(num_particles, tagMap, DT);
    pf->init(x0, sigma_pos);

    Simulator sim(STATE_DIM, pf);
    sim.run();


    return 0;
}