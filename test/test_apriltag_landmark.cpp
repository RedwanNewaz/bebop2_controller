//
// Created by redwan on 9/29/23.
//
#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include "catch2-2.7.0/catch.hpp"

#include "airlib/localization/sensors.h"


// Define a custom matcher for comparing floating-point vectors with a tolerance
template <typename T>
bool vectorsApprox(const std::vector<T>& v1, const std::vector<T>& v2, T epsilon) {
    if (v1.size() != v2.size()) {
        return false;
    }
    for (size_t i = 0; i < v1.size(); ++i) {
        if (!(Approx(v1[i]).epsilon(epsilon) == v2[i])) {
            return false;
        }
    }
    return true;
}


TEST_CASE("ApriltagLandmarks::operator()(tag2)", "[sensors::tag2]")
{
    std::unordered_map<std::string, tf::Transform> landmarks;

    tf::Transform tag3, tag2, tag7, tag4;

    tag3.setOrigin(tf::Vector3(0.81, 7.0, 1.2));
    tag2.setOrigin(tf::Vector3(1.96, 7.0, 1.2));
    tag7.setOrigin(tf::Vector3(3.11, 7.0, 1.2));
    tag4.setOrigin(tf::Vector3(4.26, 7.0, 1.2));

    landmarks["tag3"] = tag3;
    landmarks["tag2"] = tag2;
    landmarks["tag7"] = tag7;
    landmarks["tag4"] = tag4;


    ApriltagLandmarks sensor(landmarks);

    apriltag_ros::AprilTagDetectionArray msg;
    apriltag_ros::AprilTagDetection detection;
    detection.id.emplace_back(2);
    detection.size.emplace_back(0.2);
    geometry_msgs::Pose pose;
    pose.position.x = 5.2858986490557065;
    pose.position.y = -0.424604553597878;
    pose.position.z = 4.932702626340081;

    pose.orientation.x = 0.9955603570084735;
    pose.orientation.y = -0.0009331117421080643;
    pose.orientation.z = -0.06148432045051431;
    pose.orientation.w =  -0.07126277565725347;

    detection.pose.pose.pose = pose;
    msg.detections.emplace_back(detection);
    sensor.detect_tag(msg);
    std::vector<double> result;
    sensor(result);
    std::vector<double> expected{-1.7420822905, 0.8610196311, 0.7753954464, 1.6936638552};

    double epsilon = 0.001;
    REQUIRE_THAT(result, Catch::Matchers::Predicate<std::vector<double>>(
            [&expected, epsilon](const std::vector<double>& v) {
                return vectorsApprox(v, expected, epsilon);
            },
            "is approximately equal to"
    ));

}



TEST_CASE("ApriltagLandmarks::operator()(tag3)", "[sensors::tag3]")
{
    std::unordered_map<std::string, tf::Transform> landmarks;

    tf::Transform tag3, tag2, tag7, tag4;

    tag3.setOrigin(tf::Vector3(0.81, 7.0, 1.2));
    tag2.setOrigin(tf::Vector3(1.96, 7.0, 1.2));
    tag7.setOrigin(tf::Vector3(3.11, 7.0, 1.2));
    tag4.setOrigin(tf::Vector3(4.26, 7.0, 1.2));

    landmarks["tag3"] = tag3;
    landmarks["tag2"] = tag2;
    landmarks["tag7"] = tag7;
    landmarks["tag4"] = tag4;


    ApriltagLandmarks sensor(landmarks);

    apriltag_ros::AprilTagDetectionArray msg;
    apriltag_ros::AprilTagDetection detection;
    detection.id.emplace_back(3);
    detection.size.emplace_back(0.2);
    geometry_msgs::Pose pose;
    pose.position.x = 3.8075310691626396;
    pose.position.y = -0.425654686522912;
    pose.position.z = 5.294529795984024;


    pose.orientation.x = 0.9980006526835868;
    pose.orientation.y = -0.01869981479833997;
    pose.orientation.z = -0.04065898969856207;
    pose.orientation.w = -0.04463026693102683;


    detection.pose.pose.pose = pose;
    msg.detections.emplace_back(detection);
    sensor.detect_tag(msg);
    std::vector<double> result;
    sensor(result);
    std::vector<double> expected{-1.5817188829, 1.0113353721, 0.7743453135, 1.6537630887};


    // Specify the tolerance (epsilon) for floating-point comparisons
    double epsilon = 0.001;
    // Use the custom matcher to compare the vectors
    REQUIRE_THAT(result, Catch::Matchers::Predicate<std::vector<double>>(
            [&expected, epsilon](const std::vector<double>& v) {
                return vectorsApprox(v, expected, epsilon);
            },
            "is approximately equal to"
    ));
}



TEST_CASE("ApriltagLandmarks::operator()(tag4)", "[sensors::tag4]")
{
    std::unordered_map<std::string, tf::Transform> landmarks;

    tf::Transform tag3, tag2, tag7, tag4;

    tag3.setOrigin(tf::Vector3(0.81, 7.0, 1.2));
    tag2.setOrigin(tf::Vector3(1.96, 7.0, 1.2));
    tag7.setOrigin(tf::Vector3(3.11, 7.0, 1.2));
    tag4.setOrigin(tf::Vector3(4.26, 7.0, 1.2));

    landmarks["tag3"] = tag3;
    landmarks["tag2"] = tag2;
    landmarks["tag7"] = tag7;
    landmarks["tag4"] = tag4;


    ApriltagLandmarks sensor(landmarks);

    apriltag_ros::AprilTagDetectionArray msg;
    apriltag_ros::AprilTagDetection detection;
    detection.id.emplace_back(4);
    detection.size.emplace_back(0.2);

    geometry_msgs::Pose pose;
    pose.position.x = 1.2569628681346101;
    pose.position.y = -1.51368396529112;
    pose.position.z = 6.382662030307091;


    pose.orientation.x = 0.9994695519566649;
    pose.orientation.y = -0.005404770158081432;
    pose.orientation.z = 0.03168532793601692;
    pose.orientation.w = -0.005238622401821274;


    detection.pose.pose.pose = pose;
    msg.detections.emplace_back(detection);
    sensor.detect_tag(msg);
    std::vector<double> result;
    sensor(result);
    std::vector<double> expected{3.5232029451, 0.6254661991, -0.3136839653, 1.5074696713};


    // Specify the tolerance (epsilon) for floating-point comparisons
    double epsilon = 0.001;
    // Use the custom matcher to compare the vectors
    REQUIRE_THAT(result, Catch::Matchers::Predicate<std::vector<double>>(
            [&expected, epsilon](const std::vector<double>& v) {
                return vectorsApprox(v, expected, epsilon);
            },
            "is approximately equal to"
    ));
}



TEST_CASE("ApriltagLandmarks::operator()(tag7)", "[sensors::tag7]")
{
    std::unordered_map<std::string, tf::Transform> landmarks;

    tf::Transform tag3, tag2, tag7, tag4;

    tag3.setOrigin(tf::Vector3(0.81, 7.0, 1.2));
    tag2.setOrigin(tf::Vector3(1.96, 7.0, 1.2));
    tag7.setOrigin(tf::Vector3(3.11, 7.0, 1.2));
    tag4.setOrigin(tf::Vector3(4.26, 7.0, 1.2));

    landmarks["tag3"] = tag3;
    landmarks["tag2"] = tag2;
    landmarks["tag7"] = tag7;
    landmarks["tag4"] = tag4;


    ApriltagLandmarks sensor(landmarks);

    apriltag_ros::AprilTagDetectionArray msg;
    apriltag_ros::AprilTagDetection detection;
    detection.id.emplace_back(4);
    detection.size.emplace_back(0.2);

    geometry_msgs::Pose pose;
    pose.position.x = -0.48407547865406375;
    pose.position.y = -0.8693754015801309;
    pose.position.z = 5.539013424770268;


    pose.orientation.x = 0.9918321895622483;
    pose.orientation.y = -0.020814813585094395;
    pose.orientation.z = 0.10172569041149282;
    pose.orientation.w = 0.07407789949694323;


    detection.pose.pose.pose = pose;
    msg.detections.emplace_back(detection);
    sensor.detect_tag(msg);
    std::vector<double> result;
    sensor(result);
    std::vector<double> expected{4.411003153, 1.5316113507, 0.3306245984, 1.3643899208};


    // Specify the tolerance (epsilon) for floating-point comparisons
    double epsilon = 0.001;
    // Use the custom matcher to compare the vectors
    REQUIRE_THAT(result, Catch::Matchers::Predicate<std::vector<double>>(
            [&expected, epsilon](const std::vector<double>& v) {
                return vectorsApprox(v, expected, epsilon);
            },
            "is approximately equal to"
    ));
}