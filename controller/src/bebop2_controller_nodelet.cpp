//
// Created by airlab on 5/16/23.
//
#include <ros/ros.h>
#include "JoystickInterface.h"
#include "DroneController.h"


#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

namespace bebop2_controller_nodelet
{

    class DroneController : public nodelet::Nodelet
    {
    public:
        DroneController()
        {}
        virtual ~DroneController(){

        }


    private:
        virtual void onInit()
        {
            ros::NodeHandle& private_nh = getPrivateNodeHandle();
            double dt;
            ros::param::get("/dt", dt);

            timer_ = private_nh.createTimer(ros::Duration(dt), &DroneController::timerCallback, this);
        }

        void timerCallback(const ros::TimerEvent& event)
        {
            auto currentMode = joyInterface_.getButtonState();
            if(currentMode == bebop2::CONTROL)
            {
                auto setpoint = joyInterface_.getSetpoint();
                droneController_.sendControl(setpoint);
            }
        }



        bebop2::JoystickInterface joyInterface_;
        bebop2::DroneController droneController_;
        ros::Timer timer_;


    };

    PLUGINLIB_EXPORT_CLASS(bebop2_controller_nodelet::DroneController, nodelet::Nodelet)
}

