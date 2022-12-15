//
// Created by roboticslab on 12/14/22.
//

#ifndef BEBOP2_CONTROLLER_POSITIONCONTROLLER_H
#define BEBOP2_CONTROLLER_POSITIONCONTROLLER_H
#include "JoystickController.h"
#include "../localization/StateEstimation.h"

namespace bebop2
{
    class PositionController : public JoystickController{
    public:
        explicit PositionController(std::shared_ptr<StateEstimation> stateEstimation);

    private:
        std::shared_ptr<StateEstimation> stateEstimation_;
        FieldLocation setPoints_;
        ros::Timer timer_;

    protected:
        void control_loop(const ros::TimerEvent& event);

        void update_set_point(double dx, double dy, double dz) override;
    };

}

#endif //BEBOP2_CONTROLLER_POSITIONCONTROLLER_H
