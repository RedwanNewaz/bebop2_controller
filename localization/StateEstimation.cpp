//
// Created by roboticslab on 12/13/22.
//

#include "StateEstimation.h"

StateEstimation::StateEstimation() {

//    tf::Transform tag2, tag4, tag7;
//    tag2.setOrigin(tf::Vector3(5, 0,0.95));
//    tag4.setOrigin(tf::Vector3(5, -2.05,0.95));
//    tag7.setOrigin(tf::Vector3(5, -1.13,0.95));
//
//    landmarks_["tag2"] = tag2;
//    landmarks_["tag4"] = tag4;
//    landmarks_["tag7"] = tag7;

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

//    alpha_ = 0.99;
    ros::param::get("~alpha", alpha_);

    apriltagSub_ = nh_.subscribe("tag_detections", 1, &StateEstimation::apriltag_callback, this);


}

void StateEstimation::apriltag_callback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg) {

    for(auto detection: msg->detections)
    {
        std::string tagName = "tag" + std::to_string(detection.id[0]);
        auto pose = detection.pose.pose.pose;
        tf::Transform tagTransform;
        tagTransform.setOrigin(tf::Vector3(pose.position.x, pose.position.y, pose.position.z));
        tagTransform.setRotation(tf::Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w));
        auto tagStateObs = transformToGlobalFrame(tagTransform, tagName);

        // update complementary filter based on each tag
        if(robot_position_.tagName.empty())
        {
            robot_position_ = tagStateObs;
            robot_position_.tagName = "robot";
        }
        else
        {

            robot_position_ = robot_position_ * alpha_ +  tagStateObs * (1 - alpha_);
        }
    }

    // filter updated now publish message
    if(!robot_position_.tagName.empty())
    {
        tf::Transform globalCoord;
        globalCoord.setOrigin(tf::Vector3(robot_position_.x, robot_position_.y, robot_position_.z));
        globalCoord.setRotation(tf::Quaternion(0, 0, 0, 1));
        br_.sendTransform(tf::StampedTransform(globalCoord, ros::Time::now(), "map", "robot"));
        ROS_INFO_STREAM(robot_position_);
    }
}

FieldLocation StateEstimation::transformToGlobalFrame(const tf::Transform &tagTransform, const std::string& tagName) {

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

FieldLocation StateEstimation::getPosition() {
    return robot_position_;
}

