//
// Created by redwan on 9/30/23.
//

#include "airlib/control/pid/quad_pids.h"


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

        std::vector<double> rawControl(NUM_CONTROLLER);
        for (int i = 0; i < NUM_CONTROLLER; ++i) {
            //        ROS_INFO("[PositionController]: control axis = %d setpoint = %lf X = %lf", i, setPoints[i], X[i] );
            bool normalized = (i == NUM_CONTROLLER - 1);
            rawControl[i] = _quadController[i].calculate(setPoints[i], X[i], normalized);
            //yaw_error = (yaw_error + np.pi) % (2 * np.pi) - np.pi  # Normalize between -π and π
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
        control.push_back(-rawControl[3]);

    }


}
