//
// Created by redwan on 3/3/23.
//

#ifndef BEBOP2_CONTROLLER_ROBOTLOCALIZATION_H
#define BEBOP2_CONTROLLER_ROBOTLOCALIZATION_H
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "FilterBase.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "ComplementaryFilter.h"

namespace bebop2{

    class RobotLocalization: public FilterBase{
    public:
        explicit RobotLocalization(ros::NodeHandle& nh);
        void init(const std::vector<double>& X0) override;
        /// @brief Populates it with the estimated state of the system.
        /// @param state Vector of tyoe double indicating the state.
        void operator()(std::vector<double>& state);
        /// @brief Updates the command velocity of the drone by changing the linear x,y,z and angular z values.
        /// @param cmd A pointer that points to the cmd_vel topic.
        void update_cmd(const geometry_msgs::Twist::ConstPtr& cmd);
        /// @brief In this method, the input obs is converted from a std::vector to an Eigen vector z and then passed to the internal_update() method.
        /// After the internal_update method is called, the updated state estimate stored in xEst_ is copied into the output result vector.
        /// @param obs A vector of measured values
        /// @param result A vector to store the update state estimate.
        void update(const std::vector<double>& obs, std::vector<double>& result) override;

    private:
        std::vector<double> state_;
        ros::NodeHandle nh_;
        ros::Publisher pub_odom_;
        ros::Subscriber sub_odom_;
        std::unique_ptr<ComplementaryFilter> lowPassFilter_;
    protected:
        void obs_to_nav_msg(const std::vector<double>& state, nav_msgs::Odometry& odometry);
        void odometry_callback(const nav_msgs::Odometry::ConstPtr& msg);
    };

} // bebop2

#endif //BEBOP2_CONTROLLER_ROBOTLOCALIZATION_H
