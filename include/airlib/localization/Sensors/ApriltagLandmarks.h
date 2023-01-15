//
// Created by redwan on 12/15/22.
//

#ifndef airlib_APRILTAGLANDMARKS_H
#define airlib_APRILTAGLANDMARKS_H
#include <memory>
#include <unordered_map>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "airlib/robot_defs.h"
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>

#include <queue>
#include "SensorBase.h"
class ApriltagLandmarks : public SensorBase{

public:
    ApriltagLandmarks(ros::NodeHandle& nh);
    void operator()(std::vector<double>& result);
    bool empty();

private:
    std::unordered_map<std::string, tf::Transform> landmarks_;
    ros::Subscriber apriltagSub_;
    ros::NodeHandle nh_;
    std::queue<std::vector<double>> measurements_;

protected:
    void apriltag_callback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg);
    FieldLocation transformToGlobalFrame(const tf::Transform& tagTransform, const std::string& tagName);


};


#endif //airlib_APRILTAGLANDMARKS_H
