//
// Created by redwan on 1/14/23.
//

#ifndef BEBOP2_CONTROLLER_QUADCONTROLLERPID_H
#define BEBOP2_CONTROLLER_QUADCONTROLLERPID_H

#include "bebop2_controller/control/PID.h"
#include "bebop2_controller/localization/StateObserver.h"
#include "bebop2_controller/control/ControllerBase.h"
#include <vector>

namespace bebop2{
    class QuadControllerPID : public ControllerBase{
    public:
        QuadControllerPID(StateObserverPtr mGetState, ros::NodeHandle& nh);
    private:
        bebop2::PID _quadController[NUM_CONTROLLER];
        void set_gains(const std::vector<double> &gains, bebop2::PID *controller);
    protected:
        void compute_control(const std::vector<double> &X, const std::vector<double> &setPoints,
                             std::vector<double> &control) override;
    };
}


#endif //BEBOP2_CONTROLLER_QUADCONTROLLERPID_H
