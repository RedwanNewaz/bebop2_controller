//
// Created by redwan on 12/15/22.
//

#ifndef BEBOP2_CONTROLLER_APRILTAGLANDMARKS_H
#define BEBOP2_CONTROLLER_APRILTAGLANDMARKS_H
#include <memory>
#include <unordered_map>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "bebop2_controller/robot_defs.h"
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <queue>

class ApriltagLandmarks {

public:
    ApriltagLandmarks();
    void publish_tf(const std::vector<double>& state);
    std::vector<double> get_observations();
    bool empty();

private:
    std::unordered_map<std::string, tf::Transform> landmarks_;
    ros::Subscriber apriltagSub_;
    ros::NodeHandle nh_;
    tf::TransformBroadcaster br_;
    std::queue<std::vector<double>> measurements_;

protected:
    void apriltag_callback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg);
    FieldLocation transformToGlobalFrame(const tf::Transform& tagTransform, const std::string& tagName);


};


#endif //BEBOP2_CONTROLLER_APRILTAGLANDMARKS_H
