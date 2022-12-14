//
// Created by roboticslab on 12/13/22.
//

#ifndef BEBOP2_CONTROLLER_STATEESTIMATION_H
#define BEBOP2_CONTROLLER_STATEESTIMATION_H


#include <memory>
#include <unordered_map>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "bebop2_controller/robot_defs.h"
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

class StateEstimation: public std::enable_shared_from_this<StateEstimation>{
public:
    StateEstimation();


private:
    ros::NodeHandle nh_;
    tf::TransformListener listener_;

    std::unordered_map<std::string, tf::Transform> landmarks_;
    ros::Subscriber apriltagSub_;
    FieldLocation robot_position_;
    tf::TransformBroadcaster br_;
    double alpha_;


protected:
    void apriltag_callback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg);

    FieldLocation transformToGlobalFrame(const tf::Transform& tagTransform, const std::string& tagName);

};


#endif //BEBOP2_CONTROLLER_STATEESTIMATION_H
