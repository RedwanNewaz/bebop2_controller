//
// Created by redwan on 12/15/22.
//

#include "bebop2_controller/localization/Sensors/ApriltagLandmarks.h"

ApriltagLandmarks::ApriltagLandmarks(ros::NodeHandle& nh): nh_(nh) {


    std::vector<int>tagIds;
    ros::param::get("~apriltags", tagIds);
    for(auto tagId : tagIds)
    {
        std::string tag_name = "tag" + std::to_string(tagId);
        std::vector<double> value;
        ros::param::get("~" + tag_name, value);
        tf::Transform tagTransform;
        tagTransform.setOrigin(tf::Vector3(value[0], value[1], value[2]));
        landmarks_[tag_name] = tagTransform;
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
        measurements_.push({position.x, position.y, position.z, 0});
//        ROS_INFO_STREAM("[ApriltagLandmarks] mes size" << measurements_.size());
    }

}

FieldLocation ApriltagLandmarks::transformToGlobalFrame(const tf::Transform &tagTransform, const std::string &tagName) {
    // https://visp-doc.inria.fr/doxygen/visp-daily/tutorial-bebop2-vs.html
    // https://www.andre-gaschler.com/rotationconverter/

    tf::Transform endEffector;
    endEffector.setOrigin(tf::Vector3(-0.09, 0, 0));
    endEffector.setRotation(tf::Quaternion(0.5, -0.5, 0.5, -0.5));
    tf::Transform baseLink = endEffector * tagTransform;
    auto loc = landmarks_[tagName].getOrigin() - baseLink.getOrigin();
    FieldLocation tagStateObs = FieldLocation{tagName, loc.x(), loc.y(), loc.z()};

    return tagStateObs;
}


void ApriltagLandmarks::operator()(std::vector<double>& result){
    result = measurements_.front();
    measurements_.pop();
}

bool ApriltagLandmarks::empty() {
    return measurements_.empty();
}

