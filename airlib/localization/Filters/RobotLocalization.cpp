//
// Created by redwan on 3/3/23.
//

#include "airlib/localization/Filters/RobotLocalization.h"

namespace bebop2 {
    void RobotLocalization::init(const std::vector<double> &X0) {
        // copy this initial value to internal state
        std::copy(X0.begin(), X0.end(), std::back_inserter(state_));
       lowPassFilter_->init(X0);
    }

    void RobotLocalization::operator()(std::vector<double> &result) {
        if(result.empty())
            std::copy(state_.begin(), state_.end(), std::back_inserter(result));
        else
            std::copy(state_.begin(), state_.end(), result.begin());
    }

    void RobotLocalization::update_cmd(const geometry_msgs::Twist_<std::allocator<void>>::ConstPtr &cmd) {

    }

    void RobotLocalization::update(const std::vector<double> &obs, std::vector<double> &result) {
        // publish nav message for robot localization node
        nav_msgs::Odometry msg;
        obs_to_nav_msg(obs, msg);
        pub_odom_.publish(msg);
        if(result.empty())
            std::copy(state_.begin(), state_.end(), std::back_inserter(result));
        else
            std::copy(state_.begin(), state_.end(), result.begin());

    }

    void RobotLocalization::obs_to_nav_msg(const std::vector<double> &obs, nav_msgs::Odometry &msg) {
        msg.header.frame_id = "mapDummy";
        msg.header.stamp = ros::Time::now();
        msg.pose.pose.position.x = obs[0];
        msg.pose.pose.position.y = obs[1];
        msg.pose.pose.position.z = obs[2];
        // compute yaw
        tf::Quaternion q;
        q.setRPY(0, 0, obs[3]);
        msg.pose.pose.orientation.x = q.x();
        msg.pose.pose.orientation.y = q.y();
        msg.pose.pose.orientation.z = q.z();
        msg.pose.pose.orientation.w = q.w();
    }

    void RobotLocalization::odometry_callback(const nav_msgs::Odometry::ConstPtr &msg) {

        std::vector<double> temp(4);

        temp[0] = msg->pose.pose.position.x;
        temp[1] = msg->pose.pose.position.y;
        temp[2] = msg->pose.pose.position.z;

        tf::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        temp[3] = q.getAngle();

        lowPassFilter_->update(temp, state_);


    }

    RobotLocalization::RobotLocalization(ros::NodeHandle &nh) : nh_(nh){
        pub_odom_ = nh_.advertise<nav_msgs::Odometry>("/apriltag/odom", 10);
        sub_odom_ = nh_.subscribe("/odometry/filtered", 10, &RobotLocalization::odometry_callback, this);
        double alpha;
        ros::param::get("~alpha", alpha);
        lowPassFilter_ = std::make_unique<ComplementaryFilter>(alpha);
    }
} // bebop2