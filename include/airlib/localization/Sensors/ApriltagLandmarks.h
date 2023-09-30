//
// Created by redwan on 12/15/22.
//

#ifndef airlib_APRILTAGLANDMARKS_H
#define airlib_APRILTAGLANDMARKS_H
#include <memory>
#include <unordered_map>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "airlib/robot_defs.h"
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <queue>
#include "SensorBase.h"
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
class ApriltagLandmarks : public SensorBase{

public:
    using MEAS_VEC = std::vector<std::pair<std::string, tf::Transform>>;
    ApriltagLandmarks(std::unordered_map<std::string, tf::Transform>& landmarks);
    void operator()(std::vector<double>& result);
    bool empty();
    /**
     * @brief given a array of detected tags, here we calculate robot global coordinate with respect to each tag.
     * Each tag propose a noisy state of the robot when transformed it to the global frame.
     * Therefore, an appropriate state filter is used to denoise those readings and compute the robot state accurately.
     *
     * @param msg an array of detected tags
     */
    void apriltag_callback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg);

    void detect_tag(const apriltag_ros::AprilTagDetectionArray& msg);


private:
    /// @brief Known landmarks with respect to map 
    std::unordered_map<std::string, tf::Transform> landmarks_;

    /// @brief measurements_ queue is reponsible to efficiently communicate with Control Module 
    std::queue<std::vector<double>> measurements_;

    /// @brief tagRoations_
    std::vector<double> tagRoations_;

protected:
    /// @brief tag is detected with respect to camera frame which needs to be transformed to global frame through
    /// camera baselink -> global_frame 
    /// @param tagTransform tag pose with resect to camera frame
    /// @param tagName name of the tag 
    /// @return global coordinate [x, y, z]
    FieldLocation transformToGlobalFrame(const tf::Transform& tagTransform, const std::string& tagName);

    /// @brief calculate heading angle based on the nearest landmark
    double calc_heading_mindist(const MEAS_VEC& z_vec);

    /// @brief calculate heading angle by averaging all the detected heading angles
    double calc_heading_avg(const MEAS_VEC& z_vec);


};


#endif //airlib_APRILTAGLANDMARKS_H
