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
    tf::Transform camBaseLink;
    // convert camera to base_link which is a fixed coordinate and given as follows
    camBaseLink.setOrigin(tf::Vector3(-0.09, 0, 0));
    camBaseLink.setRotation(tf::Quaternion(0.5, -0.5, 0.5, -0.5));


    MEAS_VEC z_vec;

    for(auto detection: msg->detections)
    {
        std::string tagName = "tag" + std::to_string(detection.id[0]);
        auto pose = detection.pose.pose.pose;
        tf::Transform tagTransform;
        tagTransform.setOrigin(tf::Vector3(pose.position.x, pose.position.y, pose.position.z));
        tagTransform.setRotation(tf::Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w));


        tf::Transform baseLink = camBaseLink * tagTransform;
        // compute heading angle
        auto euler = getEulerFromQuat(baseLink.getRotation());
        tf::Quaternion q;
        q.setRPY(0, 0, -euler[2]);
        baseLink.setRotation(q);
        z_vec.emplace_back(tagName, baseLink);

//        static tf::TransformBroadcaster br;
//        br.sendTransform(tf::StampedTransform(mapToRobot, ros::Time::now(), "map", "map_" + tagName));
    }

    double heading = calc_heading_mindist(z_vec);

    for(const auto&z :z_vec)
    {
        tf::Transform mapToRobot(z.second);
        auto coord = transformToGlobalFrame(mapToRobot, z.first);
        mapToRobot.setOrigin(tf::Vector3(coord.x, coord.y, coord.z));

        auto q = mapToRobot.getRotation();
//        double theta = heading / (double) z_vec.size();
        double theta = heading;
        q.setRPY(0, 0, theta);
        auto ori = getEulerFromQuat(q);
        auto pos = mapToRobot.getOrigin();
        auto z_i = std::vector<double>{pos.x(), pos.y(), pos.z(), ori[2]};
        measurements_.push(z_i);
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

double ApriltagLandmarks::calc_heading_mindist(const ApriltagLandmarks::MEAS_VEC &z_vec) {
    double heading = 0.0;
    const tf2::Vector3 origin(0, 0, 0);
    double maxDist = std::numeric_limits<double>::max();
    for(const auto&z :z_vec)
    {

        auto node = z.second.getOrigin();
        const tf2::Vector3 tagOrigin(node.x(),node.y(), node.z());
        double dist = tf2::tf2Distance(origin, tagOrigin);
        if(dist < maxDist)
        {
            maxDist = dist;
            auto euler = getEulerFromQuat(z.second.getRotation());
            heading = euler[2];
        }

    }
    return heading;
}

double ApriltagLandmarks::calc_heading_avg(const ApriltagLandmarks::MEAS_VEC &z_vec) {
    double heading = 0.0;
    for(const auto&z :z_vec)
    {
        auto euler = getEulerFromQuat(z.second.getRotation());
        heading += euler[2];
    }
    return heading / (double) z_vec.size();
}
