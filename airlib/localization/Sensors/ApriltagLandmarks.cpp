//
// Created by redwan on 12/15/22.
//
#include <Eigen/Dense>
#include "airlib/localization/Sensors/ApriltagLandmarks.h"

ApriltagLandmarks::ApriltagLandmarks(ros::NodeHandle& nh): nh_(nh) {


    std::vector<int>tagIds;
    ros::param::get("/apriltags", tagIds);
    for(auto tagId : tagIds)
    {
        std::string tag_name = "tag" + std::to_string(tagId);
        std::vector<double> value;
        ros::param::get("/" + tag_name, value);

        tf::Transform tagTransform;
        auto q = tf::Quaternion(value[3], value[4], value[5], value[6]);
        tagTransform.setOrigin(tf::Vector3(value[0], value[1], value[2]));
        tagTransform.setRotation(q);
        landmarks_[tag_name] = tagTransform;
        ROS_INFO("[ApriltagLandmarks] tag = %d origin = (%lf %lf %lf) q = (%lf %lf %lf) ", tagId, value[0], value[1], value[2], q.x(), q.y(), q.z());

    }

    apriltagSub_ = nh_.subscribe("/tag_detections", 1, &ApriltagLandmarks::apriltag_callback, this);
    ROS_INFO_STREAM("[ApriltagLandmarks] initialized");
}



void
ApriltagLandmarks::apriltag_callback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg) {

//    ROS_INFO_STREAM(*msg);
    for(auto detection: msg->detections)
    {
        std::string tagName = "tag" + std::to_string(detection.id[0]);
        auto pose = detection.pose.pose.pose;
        tf::Transform tagTransform;
        tagTransform.setOrigin(tf::Vector3(pose.position.x, pose.position.y, pose.position.z));
        tagTransform.setRotation(tf::Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w));
        auto position = transformToGlobalFrame(tagTransform, tagName);
        double tagId = detection.id[0];
        measurements_.push({position.x, position.y, position.z, 0.0});

    }

}

FieldLocation ApriltagLandmarks::transformToGlobalFrame(const tf::Transform &tagTransform, const std::string &tagName) {
    // https://visp-doc.inria.fr/doxygen/visp-daily/tutorial-bebop2-vs.html
    // https://www.andre-gaschler.com/rotationconverter/

    tf::Transform endEffector;
    // convert camera to base_link which is a fixed coordinate and given as follows
    endEffector.setOrigin(tf::Vector3(-0.09, 0, 0));
    endEffector.setRotation(tf::Quaternion(0.5, -0.5, 0.5, -0.5));
    tf::Transform baseLink = endEffector * tagTransform;
    // convert base_link to map coordinate
//    auto loc = landmarks_[tagName].getOrigin() - baseLink.getOrigin();
    auto loc = landmarks_[tagName].getOrigin();
    auto rot = landmarks_[tagName].getRotation();
    auto xx = baseLink.getOrigin();

    Eigen::Vector3d p(loc.x(), loc.y(), loc.z());
    Eigen::Vector3d r(xx.x(), xx.y(), xx.z());
    Eigen::Matrix3d q;
    q.setIdentity();
    q(0, 0) = rot.x();
    q(1, 1) = rot.y();
    q(2, 2) = rot.z();

    Eigen::Vector3d T = p - q * r   ;
//    Eigen::Vector3d T = p + q * r  ;
//    FieldLocation tagStateObs = FieldLocation{tagName, loc.x(), loc.y(), loc.z()};

    FieldLocation tagStateObs = FieldLocation{tagName, T(0), T(1), T(2)};

    return tagStateObs;
}


void ApriltagLandmarks::operator()(std::vector<double>& result){
    result = measurements_.front();
//    ROS_INFO_STREAM("[ApriltagLandmarks] mes size" << measurements_.size() << " current tag = " << result[3]);
    measurements_.pop();
}

bool ApriltagLandmarks::empty() {
    return measurements_.empty();
}

