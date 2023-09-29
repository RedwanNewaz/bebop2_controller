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
        auto q = tf::Quaternion();
        auto alphaAngle = atan2(value[1], value[2]);
        q.setRPY(0, 0, alphaAngle);
        tagTransform.setOrigin(tf::Vector3(value[0], value[1], value[2]));


        tagTransform.setRotation(q);


        landmarks_[tag_name] = tagTransform;
        ROS_INFO("[ApriltagLandmarks] tag = %d origin = (%lf %lf %lf) q = (%lf %lf %lf %lf) ", tagId, value[0], value[1], value[2], q.x(), q.y(), q.z(), q.w());

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


        tf::Transform camBaseLink;
        // convert camera to base_link which is a fixed coordinate and given as follows
        camBaseLink.setOrigin(tf::Vector3(-0.09, 0, 0));
        camBaseLink.setRotation(tf::Quaternion(0.5, -0.5, 0.5, -0.5));
        tf::Transform baseLink = camBaseLink * tagTransform;

        // compute heading angle
        tf::Matrix3x3 m(baseLink.getRotation());
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        tf::Quaternion q;
        q.setRPY(0, 0, -pitch + M_PI_2);
        baseLink.setRotation(q);

        tf::Transform mapToRobot(baseLink);
        auto coord = transformToGlobalFrame(mapToRobot, tagName);
        mapToRobot.setOrigin(tf::Vector3(coord.x, coord.y, coord.z));

        auto ori = getEulerFromQuat(mapToRobot.getRotation());
        auto pos = mapToRobot.getOrigin();
        auto z_i = std::vector<double>{pos.x(), pos.y(), pos.z(), ori[2]};
        measurements_.push(z_i);

//        static tf::TransformBroadcaster br;
//        br.sendTransform(tf::StampedTransform(mapToRobot, ros::Time::now(), "map", "map_" + tagName));
    }



}

FieldLocation ApriltagLandmarks::transformToGlobalFrame(const tf::Transform &tagTransform, const std::string &tagName) {
    // https://visp-doc.inria.fr/doxygen/visp-daily/tutorial-bebop2-vs.html
    // https://www.andre-gaschler.com/rotationconverter/


    // convert base_link to map coordinate

    auto loc = landmarks_[tagName].getOrigin();
    double d = std::hypot(loc.x(), loc.y(), loc.z());
    double alpha = acos(loc.z() / d);


    auto xx = tagTransform.getOrigin();
    auto ori = getEulerFromQuat(tagTransform.getRotation());


    double theta = M_PI_2 + ori[2] - alpha;
    // convert theta -pi to pi
    theta = fmod(theta + M_PI, 2 * M_PI) - M_PI;

    double c = cos(theta);
    double s = sin(theta);

    Eigen::Vector3d p(loc.x(), loc.y(), loc.z());
    Eigen::Vector3d r(xx.x(), xx.y(), xx.z());
    Eigen::Matrix3d q;
    q.setIdentity();
    q(0, 0) = c;
    q(0, 1) = -s;
    q(1, 0) = s;
    q(1, 1) = c;

    Eigen::Vector3d T = p - q * r   ;
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
