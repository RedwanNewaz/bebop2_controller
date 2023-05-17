//
// Created by airlab on 5/16/23.
//
#include <ros/ros.h>
#include "StateEstimator.h"
#include "../../Core/StateViz.h"

#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

namespace bebop2_controller_nodelet
{

    class StateEstimator : public nodelet::Nodelet
    {
    public:
        StateEstimator()
        {}
        virtual ~StateEstimator(){

        }
        

    private:
        virtual void onInit()
        {
            ros::NodeHandle& private_nh = getPrivateNodeHandle();

            timer_ = private_nh.createTimer(ros::Duration(0.01), &StateEstimator::timerCallback, this);
        }
        
        void timerCallback(const ros::TimerEvent& event)
        {
            if(stateEstimator_.isReady())
            {
                auto state = stateEstimator_.getState();
                viz_.setDrone(state);
            }
        }



        Alse::StateEstimator stateEstimator_;
        bebop2::StateViz viz_;
        ros::Timer timer_;


    };

    PLUGINLIB_EXPORT_CLASS(bebop2_controller_nodelet::StateEstimator, nodelet::Nodelet)
}

