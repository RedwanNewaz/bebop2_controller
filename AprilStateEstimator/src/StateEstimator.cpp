//
// Created by airlab on 5/16/23.
//
#include "StateEstimator.h"

namespace Alse
{

    StateEstimator::StateEstimator() {

        std::vector<int>tagIds;
        ros::param::get("/apriltags", tagIds);
        for(auto tagId:tagIds)
        {
            std::string tag_name = "tag" + std::to_string(tagId);
            auto landmark = getLandmark(tag_name);
            m_detections[tagId] = std::make_shared<SensorData>(tagId, landmark);
            m_pub_odoms[tagId] = m_nh.advertise<nav_msgs::Odometry>("apriltag/odom" + std::to_string(tagId), 10);

        }
        m_sub = m_nh.subscribe("bebop/ekf_state", 10, &StateEstimator::ekf_subscriber, this);
        m_timer = m_nh.createTimer(ros::Duration(0.01), &StateEstimator::timerCallback, this); //. 100 Hz
        m_apriltagSub = m_nh.subscribe("/tag_detections", 1, &StateEstimator::apriltag_callback, this);
        m_isReady = false;
    }

    void StateEstimator::timerCallback(const ros::TimerEvent& event)
    {
        for(auto it:m_detections)
        {
            if(it.second->isAvailable())
            {
                auto msg = it.second->toOdomMsg();
                m_pub_odoms[it.first].publish(msg);
            }
        }
    }

    void StateEstimator::apriltag_callback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg)
    {

        int bestMatchTag = -1;
        double bestMatchValue = std::numeric_limits<double>::max();
        tf::Transform bestTransform;

        for(auto detection: msg->detections)
        {
            int tagId = detection.id[0];
            auto pose = detection.pose.pose.pose;

            auto q = tf::Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
            double roll, pitch, yaw;
            tf::Matrix3x3 m(q);
            m.getRPY(roll, pitch, yaw);
            q.setEuler(pitch, yaw, roll - M_PI);

            // convert camera frame to camera base link frame
            double x, y, z;
            x = pose.position.z - 0.09; // bebop center point offset
            y = pose.position.x;
            z = pose.position.y;
            auto curr = tf::Vector3(x, y, z);

//        // following code produces bad results
//        bestTransform.setRotation(q);
//        bestTransform.setOrigin(curr);
//        m_detections[tagId]->update_detection(bestTransform);


            // compute the minimum distance from robot to a landmark and use it as ref
            tf::Vector3 origin(0, 0, 0);
            double dist = tf::tfDistance2(curr, origin);

            if(dist < bestMatchValue)
            {
                bestTransform.setRotation(q);
                bestTransform.setOrigin(curr);
                bestMatchValue = dist;
                bestMatchTag = tagId;
            }
        }

        // if detection yields a best match tag update the sensor data
        if(m_detections.find(bestMatchTag) != m_detections.end())
        {
            m_detections[bestMatchTag]->update_detection(bestTransform);
        }

    }

    void StateEstimator::ekf_subscriber(const nav_msgs::Odometry::ConstPtr &msg) {
        auto position = msg->pose.pose.position;
        auto orientation = msg->pose.pose.orientation;
        m_state.setOrigin(tf::Vector3(position.x, position.y, position.z));
        m_state.setRotation(tf::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w));
        m_isReady = true;
    }


    tf::Transform StateEstimator::getLandmark(const std::string &tag_name) {
        std::vector<double> value;
        ros::param::get("/" + tag_name, value);

        ROS_INFO("[StateEstimator]: tagName = %s position (%lf, %lf, %lf)", tag_name.c_str(), value[0], value[1], value[2]);

        tf::Transform tagTransform;
        auto q = tf::Quaternion(value[3], value[4], value[5], value[6]);
        tagTransform.setOrigin(tf::Vector3(value[0], value[1], value[2]));
        tagTransform.setRotation(q);
        return tagTransform;
    }
}