

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
        std::string tag_name = "tag" + std::to_string(tagId);
        std::vector<double> value;
        ros::param::get("/" + tag_name, value);

        tf::Transform tagTransform;
        auto q = tf::Quaternion(value[3], value[4], value[5], value[6]);
        tagTransform.setOrigin(tf::Vector3(value[0], value[1], value[2]));
        tagTransform.setRotation(q);
        landmarks_[tag_name] = tagTransform;
        ROS_INFO("[ApriltagLandmarksExtended] tag = %d origin = (%lf %lf %lf) q = (%lf %lf %lf) ", tagId, value[0], value[1], value[2], q.x(), q.y(), q.z());

    }

    apriltagSub_ = nh_.subscribe("/tag_detections", 1, &ApriltagLandmarksExtended::apriltag_callback, this);
    ROS_INFO_STREAM("[ApriltagLandmarksExtended] initialized");
    filterCount_ = 0;
}



void
ApriltagLandmarksExtended::apriltag_callback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg) {

//    if(++filterCount_ < 10)
//        return;
//    ROS_INFO_STREAM(*msg);
    for(auto detection: msg->detections)
    {
        std::string tagName = "tag" + std::to_string(detection.id[0]);
        auto pose = detection.pose.pose.pose;
        tf::Transform tagTransform;
        tagTransform.setOrigin(tf::Vector3(pose.position.x, pose.position.y, pose.position.z));
        tagTransform.setRotation(tf::Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w));

        tf::Transform endEffector;
        // convert camera to base_link which is a fixed coordinate and given as follows
        endEffector.setOrigin(tf::Vector3(-0.09, 0, 0));
        endEffector.setRotation(tf::Quaternion(-0.5, 0.5, 0.5, 0.5 ));
//        tf::Transform baseLink = endEffector.inverseTimes(tagTransform);
        tf::Transform baseLink = tagTransform.inverseTimes(endEffector);
//        std::call_once(tagInits_[tagName], [&](){
//
//            landmarks_[tagName] = baseLink;
//
//            double roll, pitch, yaw;
//            tf::Matrix3x3 m(baseLink.getRotation());
//            m.getRPY(roll, pitch, yaw);
//            auto position = baseLink.getOrigin();
//            ROS_INFO("[ApriltagLandmarksExtended] init %s : xyz = (%lf, %lf, %lf) rpy = (%lf, %lf, %lf)",
//                     tagName.c_str(), position.x(), position.y(), position.z(), roll, pitch, yaw);
//
//        });
        // get relative pose with respect to initial position: invert one and multiply the other.
//        auto newTransform = landmarks_[tagName].inverseTimes(tagTransform);
        auto newTransform = baseLink.inverseTimes(landmarks_[tagName]);
        auto position = newTransform.getOrigin();
        double tagId = detection.id[0];
        measurements_.push({position.x(), position.y(), position.z(), tagId});

    }

}


void ApriltagLandmarksExtended::operator()(std::vector<double>& result){
    result = measurements_.front();
//    ROS_INFO_STREAM("[ApriltagLandmarksExtended] mes size" << measurements_.size() << " current tag = " << result[3]);
    measurements_.pop();
}

bool ApriltagLandmarksExtended::empty() {
    return measurements_.empty();
}

