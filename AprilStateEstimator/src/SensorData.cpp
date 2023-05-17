//
// Created by airlab on 5/16/23.
//

#include "SensorData.h"

namespace Alse
{

    SensorData::SensorData(int id, const tf::Transform& landmark) : tagId_(id),landmark_(landmark)
    {
        std::string landmarkName = "tag" + std::to_string(id);
        pub_ = nh_.advertise<nav_msgs::Odometry>("apriltag/" + landmarkName, 10);
        sub_ = nh_.subscribe("odometry/filtered/"+ landmarkName, 10, &SensorData::ekf_subscriber, this);
        ready_ = covReady_ = false;
    }

    SensorDataPtr SensorData::getPtr(){
        return shared_from_this();
    }

    SensorData::OdomMsg SensorData::toOdomMsg(const std::string &frameId) {
        std::lock_guard<std::mutex>lockGuard(mu_);

        OdomMsg msg;
        msg.header.frame_id = frameId;
        msg.header.stamp = updateTime_;

        auto obs = getData();

        msg.pose.pose.position.x = obs.getOrigin().x();
        msg.pose.pose.position.y = obs.getOrigin().y();
        msg.pose.pose.position.z = obs.getOrigin().z();
        // compute yaw
        tf::Quaternion q = obs.getRotation();

        msg.pose.pose.orientation.x = q.x();
        msg.pose.pose.orientation.y = q.y();
        msg.pose.pose.orientation.z = q.z();
        msg.pose.pose.orientation.w = q.w();


        if(covReady_)
        {
            auto cov_ = msg_.pose.covariance;
            std::copy(cov_.begin(), cov_.end(), msg.pose.covariance.begin());
        }

        // track current state uncertainty for this landmark
        pub_.publish(msg);

//    return msg;
        return covReady_?msg_:msg;
    }

    void SensorData::update_detection(const tf::Transform &tagTransform) {
        std::lock_guard<std::mutex> lk(mu_);

        detection_ = landmark_.inverseTimes(tagTransform);
        // all coordinates goes negative after above transformation but proper orientation
        auto currOri = detection_.getOrigin() * - 1; // change negative to positive coords
        detection_.setOrigin(currOri);
        ready_ = true;
        updateTime_ = ros::Time::now();

//        static tf::TransformBroadcaster br;
//        std::string frame_name = "Map2Tag" + std::to_string(tagId_);
//        br.sendTransform(tf::StampedTransform(detection_, ros::Time::now(), "map", frame_name));

    }

    tf::Transform SensorData::getData() {
        assert(ready_ && "data is not ready. use isAvailable in your code to access this method");
        ready_ = false;
        // additionally we can compute update rate here by comparing timestamp
        return detection_;
    }

    bool SensorData::isAvailable() {
        return ready_;
    }

    void SensorData::ekf_subscriber(const nav_msgs::Odometry_<std::allocator<void>>::ConstPtr &msg) {
//    ROS_INFO_STREAM( tagId_<< "\n"  << *msg);
        std::lock_guard<std::mutex> lk(mu_);
        covReady_ = true;
        msg_ = deepCopyOdometry(*msg);

    }


    nav_msgs::Odometry SensorData::deepCopyOdometry(const nav_msgs::Odometry& original) {
        // Serialize the original message
        uint32_t serializedSize = ros::serialization::serializationLength(original);
        boost::shared_array<uint8_t> buffer(new uint8_t[serializedSize]);
        ros::serialization::OStream stream(buffer.get(), serializedSize);
        ros::serialization::serialize(stream, original);

        // Deserialize the serialized data into a new message
        nav_msgs::Odometry copied;
        ros::serialization::IStream istream(buffer.get(), serializedSize);
        ros::serialization::deserialize(istream, copied);

        return copied;
    }
}