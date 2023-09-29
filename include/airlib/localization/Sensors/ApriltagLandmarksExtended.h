//
// Created by redwan on 3/3/23.
//

#ifndef BEBOP2_CONTROLLER_APRILTAGLANDMARKSEXTENDED_H
#define BEBOP2_CONTROLLER_APRILTAGLANDMARKSEXTENDED_H
#include <memory>
#include <unordered_map>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "airlib/robot_defs.h"
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <mutex>
#include <queue>
#include "SensorBase.h"
#include <mutex>
/**
 * @brief This class compute the robot coordinate from stationary tags.
 *
 * There are three stationary apriltags located on the wall.
 * These tags are treated as known landmarks for the map.
 * Bebop2 can view these tags with the front facing camera.
 * It is not necessary to see all the tags even one landmark can help to localize the robot.
 * This information is feedback to the computer to detect apriltags from the camera image.
 * Bebop2 can transmit this images upto 28 Hz speed
 * Apprantly, the tags are located only front direction, therefore robot yaw angle is ignored.
 *
 */

class SensorData;
typedef std::shared_ptr<SensorData>SensorDataPtr;
class SensorData: public std::enable_shared_from_this<SensorData>
{
public:
    using OdomMsg = nav_msgs::Odometry;
    SensorData(int id): tagId_(id)
    {
        std::string landmarkName = "tag" + std::to_string(id);
        pub_ = nh_.advertise<nav_msgs::Odometry>("/apriltag/" + landmarkName, 10);
        sub_ = nh_.subscribe("odometry/filtered/"+ landmarkName, 10, &SensorData::ekf_subscriber, this);

        landmark_ = getLandmark(landmarkName);
        ready_ = covReady_ = false;
    }

    SensorDataPtr getPtr()
    {
        return shared_from_this();
    }

    bool isAvailable()
    {
        return ready_;
    }

    tf::Transform getData()
    {
        assert(ready_ && "data is not ready. use isAvailable in your code to access this method");
        ready_ = false;
        // additionally we can compute update rate here by comparing timestamp
        return detection_;
    }

    OdomMsg toOdomMsg(const std::string& frameId = "odom")
    {
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
            std::copy(cov_.begin(), cov_.end(), msg.pose.covariance.begin());
        }

        // track current state uncertainty for this landmark
        pub_.publish(msg);


        return covReady_?msg_:msg;
    }

    void update_detection(const tf::Transform& tagTransform)
    {
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
protected:
    void ekf_subscriber(const nav_msgs::Odometry::ConstPtr& msg)
    {
        ROS_INFO_STREAM( tagId_<< "\n"  << *msg);
        covReady_ = true;
        auto cov = msg->pose.covariance;
        std::copy(cov.begin(), cov.end(), cov_.begin());
        msg_ = *msg;

    }

    tf::Transform getLandmark(const std::string& tag_name)
    {
        std::vector<double> value;
        ros::param::get("/" + tag_name, value);

        tf::Transform tagTransform;
        auto q = tf::Quaternion(value[3], value[4], value[5], value[6]);
        tagTransform.setOrigin(tf::Vector3(value[0], value[1], value[2]));
        tagTransform.setRotation(q);
        return tagTransform;
    }
private:
    tf::Transform landmark_;
    tf::Transform detection_;
    bool ready_;
    bool covReady_;
    int tagId_;
    ros::Time updateTime_;
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    std::array<double, 36> cov_;
    nav_msgs::Odometry msg_;

};




class ApriltagLandmarksExtended : public SensorBase{

public:
    ApriltagLandmarksExtended(ros::NodeHandle& nh);
    void operator()(std::vector<double>& result);
    bool empty();

private:
    /// @brief Known landmarks with respect to map
    std::unordered_map<std::string, tf::Transform> landmarks_;
    /// @brief apriltag node subscriber
    ros::Subscriber apriltagSub_;
    /// @brief ros NodeHandle for dealing with various messages
    ros::NodeHandle nh_;
    /// @brief measurements_ queue is reponsible to efficiently communicate with Control Module
    std::vector<double> measurements_;

    /// @brief tagRoations_
    std::vector<double> tagRoations_;
    ///@brief number of initialization required
    std::unordered_map<std::string, std::once_flag> tagInits_;
    ///@brief filter out some init frames
    int filterCount_;

    std::unordered_map<int, SensorDataPtr> detections_;
    ros::Timer timer_;
    ros::Subscriber sub_;
    std::unordered_map<int, ros::Publisher> m_pub_odoms;
    tf::TransformListener listener_;
    std::mutex mu_;


protected:
    /**
     * @brief given a array of detected tags, here we calculate robot global coordinate with respect to each tag.
     * Each tag propose a noisy state of the robot when transformed it to the global frame.
     * Therefore, an appropriate state filter is used to denoise those readings and compute the robot state accurately.
     *
     * @param msg an array of detected tags
     */
    void apriltag_callback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg);
    /// @brief tag is detected with respect to camera frame which needs to be transformed to global frame through
    /// camera baselink -> global_frame
    /// @param tagTransform tag pose with resect to camera frame
    /// @param tagName name of the tag
    /// @return global coordinate [x, y, z]
    void timerCallback(const ros::TimerEvent& event);

    void ekf_subscriber(const nav_msgs::Odometry::ConstPtr& msg);


};



#endif //BEBOP2_CONTROLLER_APRILTAGLANDMARKSEXTENDED_H
