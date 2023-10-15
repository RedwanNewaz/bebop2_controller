//
// Created by redwan on 3/5/23.
//
//
// Created by roboticslab on 12/13/22.
//
#include "ros/ros.h"
#include <iostream>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>
#include <future>
#include <thread>

#include "../sensor/landmark/ApriltagLandmarks.h"
#include "../filter/LPF/ComplementaryFilter.h"
#include "StateObserver.h"

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "airlib_bebop2");
    ROS_INFO("airlib-bebop2 INITIALIZED!");
    ros::NodeHandle nh;
    double alpha;
    ros::param::get("/alpha", alpha);
    std::vector<int>tagIds;
    ros::param::get("/apriltags", tagIds);
    std::unordered_map<std::string, tf::Transform> landmarks;
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
        landmarks[tag_name] = tagTransform;
    }


    ROS_INFO_STREAM("[ApriltagLandmarks] initialized");
    auto pub = nh.advertise<nav_msgs::Odometry>("apriltag/state", 10);

    auto stateSensor = std::make_shared<ApriltagLandmarks>(landmarks);
    auto stateFilter = std::make_shared<ComplementaryFilter>(alpha);
    auto stateObserver = std::make_shared<bebop2::StateObserver>(stateFilter, stateSensor);

    auto sub = nh.subscribe("/tag_detections", 10,  &ApriltagLandmarks::apriltag_callback, stateSensor.get());

    ros::AsyncSpinner spinner(4);
    spinner.start();

    std::thread stateThread = std::thread ([&]{
        try {
            // code that may throw
            while(ros::ok())
            {
                std::promise<std::vector<double>> promise;
                stateObserver->getState(std::ref(promise));
                auto xEst = promise.get_future();
                auto state = xEst.get();
                ROS_INFO("[State] (%lf, %lf, %lf, %lf)", state[0], state[1], state[2], state[3]);

                nav_msgs::Odometry odom;
                odom.header.stamp = ros::Time::now();
                odom.header.frame_id = "map";

                odom.pose.pose.position.x = state[0];
                odom.pose.pose.position.y = state[1];
                odom.pose.pose.position.z = state[2];

                tf::Quaternion q;
                q.setRPY(0, 0, state[3]);
                odom.pose.pose.orientation.x = q.x();
                odom.pose.pose.orientation.y = q.y();
                odom.pose.pose.orientation.z = q.z();
                odom.pose.pose.orientation.w = q.w();

                pub.publish(odom);

            }
        } catch(...) {

        }
    });

    ros::waitForShutdown();

    return 0;
}
