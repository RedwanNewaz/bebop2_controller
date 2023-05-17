//
// Created by airlab on 5/16/23.
//

#ifndef BEBOP2_CONTROLLER_NODELET_SENSORDATA_H
#define BEBOP2_CONTROLLER_NODELET_SENSORDATA_H


#include <ros/ros.h>
#include <memory>
#include <queue>
#include <mutex>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <unordered_map>
#include "apriltag_ros/AprilTagDetectionArray.h"
#include <nav_msgs/Odometry.h>
#include <ros/serialization.h>

//namespace apriltag landmark based state estimator Alse

namespace Alse
{


    class SensorData;
    typedef std::shared_ptr<SensorData>SensorDataPtr;

    class SensorData: public std::enable_shared_from_this<SensorData>
    {
    public:
        using OdomMsg = nav_msgs::Odometry;
        explicit SensorData(int id, const tf::Transform& landmark);
        SensorDataPtr getPtr();
        bool isAvailable();
        tf::Transform getData();
        OdomMsg toOdomMsg(const std::string& frameId = "odom");
        void update_detection(const tf::Transform& tagTransform);

    protected:
        void ekf_subscriber(const nav_msgs::Odometry::ConstPtr& msg);
        nav_msgs::Odometry deepCopyOdometry(const nav_msgs::Odometry& original);



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
        nav_msgs::Odometry msg_;
        std::mutex mu_;
    };

}


#endif //BEBOP2_CONTROLLER_NODELET_SENSORDATA_H
