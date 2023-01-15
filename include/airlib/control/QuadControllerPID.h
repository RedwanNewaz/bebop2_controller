//
// Created by redwan on 1/14/23.
//

#ifndef airlib_QUADCONTROLLERPID_H
#define airlib_QUADCONTROLLERPID_H

#include "airlib/control/PID.h"
#include "airlib/localization/StateObserver.h"
#include "airlib/control/ControllerBase.h"
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


#endif //airlib_QUADCONTROLLERPID_H
