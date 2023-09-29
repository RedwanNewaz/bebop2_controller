

//
// Created by redwan on 12/15/22.
//

#include <Eigen/Dense>
#include "airlib/localization/Sensors/ApriltagLandmarksExtended.h"

ApriltagLandmarksExtended::ApriltagLandmarksExtended(ros::NodeHandle& nh): nh_(nh) {


    std::vector<int>tagIds;
    ros::param::get("/apriltags", tagIds);
    for(auto tagId : tagIds)
    {
        detections_[tagId] = std::make_shared<SensorData>(tagId);
        m_pub_odoms[tagId] = nh_.advertise<nav_msgs::Odometry>("/apriltag/odom" + std::to_string(tagId), 10);
    }

    apriltagSub_ = nh_.subscribe("/tag_detections", 1, &ApriltagLandmarksExtended::apriltag_callback, this);
    ROS_INFO_STREAM("[ApriltagLandmarksExtended] initialized");
    filterCount_ = 0;
    timer_ = nh_.createTimer(ros::Duration(0.01), &ApriltagLandmarksExtended::timerCallback, this); //. 100 Hz
    sub_ = nh_.subscribe("/odometry/filtered", 10, &ApriltagLandmarksExtended::ekf_subscriber, this);
}

void ApriltagLandmarksExtended::ekf_subscriber(const nav_msgs::Odometry::ConstPtr& msg)
{
    const std::lock_guard<std::mutex> lock(mu_);
    auto position = msg->pose.pose.position;
    auto orientation = msg->pose.pose.orientation;

    auto q = tf::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w);
    double roll, pitch, yaw;
    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);
    measurements_.clear();
    measurements_.push_back(position.x);
    measurements_.push_back(position.y);
    measurements_.push_back(position.z);
    measurements_.push_back(yaw);

}

void ApriltagLandmarksExtended::timerCallback(const ros::TimerEvent& event)
{
    for(auto it:detections_)
    {
        if(it.second->isAvailable())
        {
            auto msg = it.second->toOdomMsg();
            m_pub_odoms[it.first].publish(msg);
        }
    }

//
//    tf::StampedTransform transform;
//    bool isStateAvailable = false;
//    try{
//        listener_.lookupTransform("map", "ekf/base_link ",
//                                 ros::Time(0), transform);
//        isStateAvailable = true;
//    }
//    catch (tf::TransformException ex){
////        ROS_ERROR("%s",ex.what());
////        ros::Duration(1.0).sleep();
//    }
//
//    if(isStateAvailable)
//    {
//        auto position = transform.getOrigin();
//        auto q = transform.getRotation();
//        double roll, pitch, yaw;
//        tf::Matrix3x3 m(q);
//        m.getRPY(roll, pitch, yaw);
////        measurements_.push({position.x(), position.y(), position.z(), yaw});
//    }
}

void
ApriltagLandmarksExtended::apriltag_callback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg) {

//    if(++filterCount_ < 10)
//        return;
//    ROS_INFO_STREAM(*msg);
    for(auto detection: msg->detections)
    {
        int tagId = detection.id[0];
        auto pose = detection.pose.pose.pose;
        tf::Transform tagTransform;

        auto q = tf::Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
        double roll, pitch, yaw;
        tf::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);
        q.setEuler(pitch, yaw, roll - M_PI);
        tagTransform.setRotation(q);

        // convert camera frame to camera base link frame
        double x, y, z;
        x = pose.position.z - 0.09; // bebop center point offset
        y = pose.position.x;
        z = pose.position.y;
        auto ori = tf::Vector3(x, y, z);
        tagTransform.setOrigin(ori);
        detections_[tagId]->update_detection(tagTransform);
//        measurements_.push({position.x(), position.y(), position.z(), tagId});

    }

}


void ApriltagLandmarksExtended::operator()(std::vector<double>& result){

    if(result.empty() && !measurements_.empty())
    {
        std::copy(measurements_.begin(), measurements_.end(), std::back_inserter(result));

    }
    else if(!result.empty())
    {
        std::copy(measurements_.begin(), measurements_.end(), result.begin());

    }

    measurements_.clear();

//    result = measurements_.front();
////    ROS_INFO_STREAM("[ApriltagLandmarksExtended] mes size" << measurements_.size() << " current tag = " << result[3]);
//    measurements_.pop();
}

bool ApriltagLandmarksExtended::empty() {
    return measurements_.empty();
}

