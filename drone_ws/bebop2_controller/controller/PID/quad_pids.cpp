//
// Created by redwan on 9/30/23.
//

#include "quad_pids.h"


namespace controller
{
    quad_pids::quad_pids(const std::vector<double> &gains, double dt) {

        assert(gains.size() == NUM_GAINS * NUM_CONTROLLER && "inaccurate PID gains");
        const double MAX_OUT = 1;
        const double MIN_OUT = -1;

        for (int i = 0; i < NUM_CONTROLLER; ++i) {
            int j = i * NUM_GAINS;
            _quadController[i].init(dt, MAX_OUT, MIN_OUT, gains[j], gains[j + 1], gains[j + 2]);
        }
    }


    void quad_pids::compute_control(const std::vector<double> &X, const std::vector<double> &setPoints,
                                            std::vector<double> &control) {

        control.resize(NUM_CONTROLLER);
        for (int i = 0; i < NUM_CONTROLLER; ++i) {
            //        ROS_INFO("[PositionController]: control axis = %d setpoint = %lf X = %lf", i, setPoints[i], X[i] );
            bool normalized = (i == NUM_CONTROLLER - 1);
            control[i] = _quadController[i].calculate(setPoints[i], X[i], normalized);
            //yaw_error = (yaw_error + np.pi) % (2 * np.pi) - np.pi  # Normalize between -π and π
        }
        // transform control to body frame
        ControllerBase::compute_control(X, setPoints, control);

    }


}
