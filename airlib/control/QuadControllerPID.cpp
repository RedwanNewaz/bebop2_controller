//
// Created by redwan on 1/14/23.
//

#include "airlib/control/QuadControllerPID.h"

namespace bebop2
{
    QuadControllerPID::QuadControllerPID(bebop2::StateObserverPtr mGetState, ros::NodeHandle &nh): ControllerBase(mGetState, nh) {
        std::vector<double> gains;
        ros::param::get("/pid_gains", gains);
        assert(gains.size() == NUM_GAINS * NUM_CONTROLLER && "inaccurate PID gains");
        set_gains(gains, _quadController);
    }

    void QuadControllerPID::set_gains(const std::vector<double> &gains, bebop2::PID *controller) {
        const double MAX_OUT = 1;
        const double MIN_OUT = -1;

        for (int i = 0; i < NUM_CONTROLLER; ++i) {
            int j = i * NUM_GAINS;
            controller[i].init(ControllerBase::dt_, MAX_OUT, MIN_OUT, gains[j], gains[j + 1], gains[j + 2]);
        }
    }

    void QuadControllerPID::compute_control(const std::vector<double> &X, const std::vector<double> &setPoints,
                                            std::vector<double> &control) {
        std::lock_guard<std::mutex> lk(ControllerBase::m_mu);
        for (int i = 0; i < NUM_CONTROLLER; ++i) {
            //        ROS_INFO("[PositionController]: control axis = %d setpoint = %lf X = %lf", i, setPoints[i], X[i] );
            bool normalized = (i == NUM_CONTROLLER - 1);
            control[i] = _quadController[i].calculate(setPoints[i], X[i], normalized);
            //yaw_error = (yaw_error + np.pi) % (2 * np.pi) - np.pi  # Normalize between -π and π

        }

    }


}
