//
// Created by roboticslab on 12/13/22.
//
#include "ros/ros.h"

#include <iostream>

#include "../localization/StateEstimation.h"
#include "../control/PID/PID.h"
#include "../control/ControllerBase.h"

namespace bebop2
{
    class QuadController: public ControllerBase
    {
    public:
        explicit QuadController(const StateFunc &mGetState) : ControllerBase(mGetState) {
            std::vector<double> gains;
            ros::param::get("~pid_gains", gains);
            assert(gains.size() == 3 * 4 && "inaccurate PID gains");
            set_gains(gains, quadController_);
        }

    private:
        bebop2::PID quadController_[NUM_CONTROLLER];
        void set_gains(const std::vector<double> &gains, bebop2::PID *controller) {
            const double MAX_OUT = 1;
            const double MIN_OUT = -1;

            for (int i = 0; i < NUM_CONTROLLER; ++i) {
                int j = i * NUM_GAINS;
                controller[i].init(m_dt, MAX_OUT, MIN_OUT, gains[j], gains[j + 1], gains[j + 2]);
            }
        }
    protected:
        void compute_control(const std::vector<double> &X, const std::vector<double> &setPoints,
                             std::vector<double> &control) override {
            std::lock_guard<std::mutex> lk(m_mu);
            for (int i = 0; i < NUM_CONTROLLER; ++i) {
                //        ROS_INFO("[PositionController]: control axis = %d setpoint = %lf X = %lf", i, setPoints[i], X[i] );
                control[i] = quadController_[i].calculate(setPoints[i], X[i]);
            }
        }


    };
}



int main(int argc, char* argv[])
{
    ros::init(argc, argv, "bebop2_controller");
    ROS_INFO("BEBOP2 CONTROLLER INITIALIZED!");


    StateEstimation stateEstimation;
    bebop2::StateFunc f = [&](){
        auto current = stateEstimation.getPosition();
        std::vector<double> x;
        if(!current.tagName.empty())
        {
            x.push_back(current.x);
            x.push_back(current.y);
            x.push_back(current.z);
            x.push_back(0);
        }
        return x;
    };

    bebop2::QuadController controller(f);


    ros::MultiThreadedSpinner spinner(2);
    spinner.spin();

    return 0;
}