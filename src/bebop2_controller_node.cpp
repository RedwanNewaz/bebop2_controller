//
// Created by roboticslab on 12/13/22.
//
#include "ros/ros.h"

#include <iostream>

#include "../control/PID/PID.h"
#include "../control/ControllerBase.h"
#include "../control/ControllerBase.cpp"
#include "../localization/Filters/ComplementaryFilter.h"
#include "../localization/Sensors/ApriltagLandmarks.h"



namespace bebop2
{
    template<class Sensor, class Filter>
    class QuadController: public ControllerBase<Sensor, Filter>
    {
    public:
        explicit QuadController(StatePtr<Sensor, Filter> mGetState, ros::NodeHandle& nh) : ControllerBase<Sensor, Filter>(mGetState, nh) {
            std::vector<double> gains;
            ros::param::get("~pid_gains", gains);
            assert(gains.size() == NUM_GAINS * NUM_CONTROLLER && "inaccurate PID gains");
            set_gains(gains, _quadController);
        }

    private:
        bebop2::PID _quadController[NUM_CONTROLLER];
        void set_gains(const std::vector<double> &gains, bebop2::PID *controller);
    protected:
        void compute_control(const std::vector<double> &X, const std::vector<double> &setPoints,
                             std::vector<double> &control) override;

    };

    template<class Sensor, class Filter>
    void QuadController<Sensor, Filter>::set_gains(const std::vector<double> &gains, bebop2::PID *controller) {
        const double MAX_OUT = 1;
        const double MIN_OUT = -1;

        for (int i = 0; i < NUM_CONTROLLER; ++i) {
            int j = i * NUM_GAINS;
            controller[i].init(ControllerBase<Sensor, Filter>::m_dt, MAX_OUT, MIN_OUT, gains[j], gains[j + 1], gains[j + 2]);
        }

    }

    template<class Sensor, class Filter>
    void
    QuadController<Sensor, Filter>::compute_control(const std::vector<double> &X, const std::vector<double> &setPoints,
                                                    std::vector<double> &control) {
        std::lock_guard<std::mutex> lk(ControllerBase<Sensor, Filter>::m_mu);
        for (int i = 0; i < NUM_CONTROLLER; ++i) {
            //        ROS_INFO("[PositionController]: control axis = %d setpoint = %lf X = %lf", i, setPoints[i], X[i] );
            control[i] = _quadController[i].calculate(setPoints[i], X[i]);
        }
    }


}



int main(int argc, char* argv[])
{
    ros::init(argc, argv, "bebop2_controller");
    ROS_INFO("BEBOP2 CONTROLLER INITIALIZED!");
    ros::NodeHandle nh;
    double alpha;
    ros::param::get("~alpha", alpha);

    auto stateFilter = std::make_shared<ComplementaryFilter>(alpha);
    auto stateSensor = std::make_shared<ApriltagLandmarks>(nh);
    auto stateObserver = std::make_shared<bebop2::StateObserver<ApriltagLandmarks, ComplementaryFilter>> (stateFilter, stateSensor);


    bebop2::QuadController<ApriltagLandmarks, ComplementaryFilter> controller(stateObserver, nh);
    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::waitForShutdown();

//    ros::MultiThreadedSpinner spinner(2);
//    spinner.spin();
    return 0;
}