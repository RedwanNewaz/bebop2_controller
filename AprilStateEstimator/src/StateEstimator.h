//
// Created by airlab on 5/16/23.
//

#ifndef BEBOP2_CONTROLLER_NODELET_STATEESTIMATOR_H
#define BEBOP2_CONTROLLER_NODELET_STATEESTIMATOR_H
#include <iostream>
#include <memory>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include "SensorData.h"
using namespace std;
//namespace apriltag landmark based state estimator: Alse

namespace Alse
{

    class StateEstimator : public enable_shared_from_this<StateEstimator>{
    public:
        using StatePtr = std::shared_ptr<StateEstimator>;
        StateEstimator();

        bool isReady() const
        {
            return m_isReady;
        }

        tf::Transform getState() const
        {
            return m_state;
        }

    protected:
        void timerCallback(const ros::TimerEvent& event);
        void apriltag_callback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg);
        void ekf_subscriber(const nav_msgs::Odometry::ConstPtr& msg);
        tf::Transform getLandmark(const std::string& tag_name);

    private:
        tf::Transform m_state;
        std::unordered_map<std::string, tf::Transform> m_Landmarks;
        ros::NodeHandle m_nh;
        ros::Subscriber m_apriltagSub;
        std::unordered_map<int, ros::Publisher> m_pub_odoms;
        std::unordered_map<int, SensorDataPtr> m_detections;
        ros::Timer m_timer;
        ros::Subscriber m_sub;
        bool m_isReady;

    };

}


#endif //BEBOP2_CONTROLLER_NODELET_STATEESTIMATOR_H
