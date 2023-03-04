//
// Created by redwan on 3/3/23.
//

#include "airlib/localization/Filters/RobotLocalization.h"

namespace bebop2 {
    void RobotLocalization::init(const std::vector<double> &X0) {
        // copy this initial value to internal state
        std::copy(X0.begin(), X0.end(), state_.begin());
       lowPassFilter_->init(state_);
    }

    void RobotLocalization::operator()(std::vector<double> &result) {
        if(result.empty())
            std::copy(state_.begin(), state_.end(), std::back_inserter(result));
        else
            std::copy(state_.begin(), state_.end(), result.begin());
    }

    void RobotLocalization::update_cmd(const geometry_msgs::Twist_<std::allocator<void>>::ConstPtr &msg) {
        state_[4] = msg->linear.x;
        state_[5] = msg->linear.y;
        state_[6] = msg->linear.z;
        state_[7] = msg->angular.z;
    }

    void RobotLocalization::update(const std::vector<double> &obs, std::vector<double> &result) {
        // publish nav message for robot localization node
        nav_msgs::Odometry msg;
        obs_to_nav_msg(obs, msg);
        pub_odom_.publish(msg);
        if(result.empty())
            std::copy(state_.begin(), state_.end(), std::back_inserter(result));
        else
            std::copy(state_.begin(), state_.begin() + result.size(), result.begin());

    }

    void RobotLocalization::obs_to_nav_msg(const std::vector<double> &obs, nav_msgs::Odometry &msg) {
        // this function is responsible to update the ekf from robot_localization package
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
        // update velocities
        msg.twist.twist.linear.x = state_[4];
        msg.twist.twist.linear.y = state_[5];
        msg.twist.twist.linear.z = state_[6];
        msg.twist.twist.angular.z = state_[7];
    }

    void RobotLocalization::odometry_callback(const nav_msgs::Odometry::ConstPtr &msg, std::vector<double>& temp) {
        // this function is invoked by two subscribers

        if(temp.empty() || temp.size() < STATE_DIM)
            temp.resize(STATE_DIM); // with default value 0

        temp[0] = msg->pose.pose.position.x;
        temp[1] = msg->pose.pose.position.y;
        temp[2] = msg->pose.pose.position.z;

        tf::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        temp[3] = q.getAngle();

    }

    RobotLocalization::RobotLocalization(ros::NodeHandle &nh) : nh_(nh){
        pub_odom_ = nh_.advertise<nav_msgs::Odometry>("apriltag/odom", 10);
        state_.resize(STATE_DIM, 0);

        // read yaw angle from bebop onboard odom parameter
        sub_odom_=nh_.subscribe<nav_msgs::Odometry> ("odom",1, [&](const nav_msgs::Odometry::ConstPtr& msg){
            odometry_callback(msg, odom_state_);
            // update yaw only
            state_[3] = odom_state_[3];
//            ROS_INFO("[RobotLocalization] yaw = %lf", state_[3]);
        });

        // read robot_localization ekf output
        sub_ekf_=nh_.subscribe<nav_msgs::Odometry> ("odometry/filtered",1, [&](const nav_msgs::Odometry::ConstPtr& msg){
            // don't change yaw angle it will create unwanted rotation
            double yaw_angle = state_[3];
            odometry_callback(msg, ekf_state_);
            lowPassFilter_->update(ekf_state_, state_);
            // use the previous value of yaw
            state_[3] = yaw_angle;
        });

        // use cmd velocities to update the observation for the ekf
        sub_cmd_ = nh_.subscribe<geometry_msgs::Twist> ("cmd_vel",1, [&](const geometry_msgs::Twist::ConstPtr& msg){
            state_[4] = msg->linear.x;
            state_[5] = msg->linear.y;
            state_[6] = msg->linear.z;
            state_[7] = msg->angular.z;
        });

        double alpha;
        ros::param::get("~alpha", alpha);
        lowPassFilter_ = std::make_unique<ComplementaryFilter>(alpha);
    }
} // bebop2