//
// Created by roboticslab on 12/14/22.
//

#ifndef BEBOP2_CONTROLLER_POSITIONCONTROLLER_H
#define BEBOP2_CONTROLLER_POSITIONCONTROLLER_H
#include "JoystickController.h"
#include "../localization/StateEstimation.h"
#include "PID/PID.h"
#define NUM_CONTROLLER  4
#define NUM_GAINS 3

namespace bebop2
{
    class PositionController : public JoystickController{
    public:
        explicit PositionController(std::shared_ptr<StateEstimation> stateEstimation);

    private:
        std::shared_ptr<StateEstimation> stateEstimation_;
        FieldLocation setPoints_;
        ros::Timer timer_;
        double goal_thres_;
        double dt_;
        bebop2::PID m_quadController[NUM_CONTROLLER];


    protected:
        void control_loop(const ros::TimerEvent& event);

        void update_set_point(double dx, double dy, double dz) override;

        void set_gains(const std::vector<double>& gains, bebop2::PID* controller);

        double goal_distance(const std::vector<double>& X, const std::vector<double>& setPoints);

        bool compute_control(const std::vector<double>& X, const std::vector<double>& setPoints, bebop2::PID* controller, std::vector<double>& control);

    };

}

#endif //BEBOP2_CONTROLLER_POSITIONCONTROLLER_H
