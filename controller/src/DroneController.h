//
// Created by airlab on 5/17/23.
//

#ifndef BEBOP2_CONTROLLER_NODELET_DRONECONTROLLER_H
#define BEBOP2_CONTROLLER_NODELET_DRONECONTROLLER_H
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include "pid.h"
#define NUM_CONTROLLER (4)
#define NUM_GAINS (3)
namespace bebop2 {

    class DroneController {
    public:
        DroneController();
        void sendControl(const std::vector<double> &setPoints);
    protected:
        void set_gains(const std::vector<double> &gains, pid *controller);

        void compute_control(const std::vector<double> &X, const std::vector<double> &setPoints,
                                              std::vector<double> &control);

        /// @brief This function sets the linear positon of the drone to all 3 linear axes and to angular z-axis and publishes the message of the current position of the drone.
        /// @param U is array of commands of size 4 which sets the above mentioned position of the drone.
        void publish_cmd_vel(const std::vector<double>&U);
        /// @brief This function calculates the distance between the setpoint of the drone and the current state position.
        /// @param X is an array of vectors of data type double representing the present position of the drone.
        /// @param setPoints is an array of vectors representing the commanded position.
        /// @return Errors between the current and commanded position of the robot.
        double goal_distance(const std::vector<double>& X, const std::vector<double>& setPoints);


        void ekf_subscriber(const nav_msgs::Odometry::ConstPtr& msg);





    private:
        /// This ROS Handle attribute is used to create various publishers and subscribers which also deals with messages.
        ros::NodeHandle nh_;
        /// Subscribes bebop2 state from nav_msg/Odometry and calls the ekf_subscriber() as a callback function.
        ros::Subscriber state_sub_;
        /// Publish cmd_vel which is geometry_msg/Twist msg to control bebop2
        ros::Publisher cntrl_pub_;
        /// drone needs 4 axis pid controller
        pid _axisController[NUM_CONTROLLER];
        /// controller sample time
        double dt_;
        /// goal region threshold
        double goalRadius_;
        /// thread safety
        std::mutex mu_;
        /// robot state
        std::vector<double> state_;

    };

} // bebop2

#endif //BEBOP2_CONTROLLER_NODELET_DRONECONTROLLER_H
