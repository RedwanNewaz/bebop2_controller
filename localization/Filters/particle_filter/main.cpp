#include "helper.h"
#include "particle_filter.h"
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

int main() {

    //Set up parameters here
    int num_particles  = 1000;
    const int STATE_DIM = 4;

    /*
     * Sigmas - just an estimate, usually comes from uncertainty of sensor, but
     * if you used fused data from multiple sensors, it's difficult to find
     * these uncertainties directly.
     */
    std::vector<double> sigma_pos{0.015, 0.015, 0.015, 0.01}; // GPS measurement uncertainty [x [m], y [m], z [m], theta [rad]]

    TagMap tagMap;
    tagMap.landmark_list.push_back({0, 5, 0, 0.95});
    tagMap.landmark_list.push_back({1, 5, -2.05, 0.95});
    tagMap.landmark_list.push_back({2, 5, -1.13, 0.95});



    size_t SIM_TIME = 0;


    StateSpace quadState;
    ParticleFilter pf(num_particles, tagMap);
    pf.init(0, 0, 1, 0, sigma_pos);
    const int MAX_ITER = 4 * 690;


//    std::ofstream dataFile;
//    dataFile.open("result.csv");

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
        pf.update_cmd(cmd);

        std::vector<double> state_filtered(STATE_DIM, 0.0);
        pf.update(state_obs, state_filtered);


//        std::cout << "[SimTime] " << SIM_TIME << " \t " << *cmd << std::endl;
//        std::copy(state_filtered.begin(), state_filtered.end(), std::ostream_iterator<double>(std:: cout, "\t"));
//        std::cout << std::endl;


        xx[SIM_TIME] = state_obs[0];
        yy[SIM_TIME] = state_obs[1];

        xf[SIM_TIME] = state_filtered[0];
        yf[SIM_TIME] = state_filtered[1];

        std::cout << "Simulation Step remaining = " << MAX_ITER - SIM_TIME << std::endl;

        delete cmd;

    }while (++SIM_TIME <= MAX_ITER);

//    dataFile.close();
    plt::scatter(xx, yy);
    plt::scatter(xf, yf);
    plt::show();
    return 0;
}