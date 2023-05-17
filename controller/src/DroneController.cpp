//
// Created by airlab on 5/17/23.
//

#include "DroneController.h"

namespace bebop2 {

    DroneController::DroneController()
    {
        ros::param::get("/dt", dt_);
        ros::param::get("/goal_thres", goalRadius_);
        std::vector<double> gains;
        ros::param::get("/pid_gains", gains);
        assert(gains.size() == NUM_GAINS * NUM_CONTROLLER && "inaccurate PID gains");
        set_gains(gains, _axisController);

        state_sub_ = nh_.subscribe("/bebop/ekf_state", 10, &DroneController::ekf_subscriber, this);
        cntrl_pub_ = nh_.advertise<geometry_msgs::Twist>("/bebop/cmd_vel", 1);
    }

    void DroneController::sendControl(const std::vector<double> &setPoints)
    {
        std::vector<double> control(NUM_CONTROLLER, 0.0);
        if(state_.empty() || setPoints.empty() || goal_distance(state_, setPoints) <= goalRadius_)
            publish_cmd_vel(control);
        else
        {
            compute_control(state_, setPoints, control);
            publish_cmd_vel(control);
            ROS_INFO("[DroneController] sending (%lf, %lf, %lf, %lf)", control[0], control[1], control[2], control[3]);
        }

    }

    void DroneController::set_gains(const std::vector<double> &gains, pid *controller) {
        std::lock_guard<std::mutex> lockGuard(mu_);
        const double MAX_OUT = 1;
        const double MIN_OUT = -1;

        for (int i = 0; i < NUM_CONTROLLER; ++i) {
            int j = i * NUM_GAINS;
            controller[i].init(dt_, MAX_OUT, MIN_OUT, gains[j], gains[j + 1], gains[j + 2]);
        }
    }

    void DroneController::compute_control(const std::vector<double> &X, const std::vector<double> &setPoints,
                                            std::vector<double> &control) {
        std::lock_guard<std::mutex> lockGuard(mu_);
        for (int i = 0; i < NUM_CONTROLLER; ++i) {
            //        ROS_INFO("[PositionController]: control axis = %d setpoint = %lf X = %lf", i, setPoints[i], X[i] );
            control[i] = _axisController[i].calculate(setPoints[i], X[i]);
        }

    }



    void DroneController::publish_cmd_vel(const std::vector<double> &U) {

        std::lock_guard<std::mutex> lk(mu_);
        assert(U.size() == 4 && "4 commands must be sent");
        geometry_msgs::Twist msg;

        msg.linear.x  = U[0];
        msg.linear.y  = U[1];
        msg.linear.z  = U[2];
        msg.angular.z = U[3];
        cntrl_pub_.publish(msg);

    }


    double DroneController::goal_distance(const std::vector<double> &X, const std::vector<double> &setPoints) {
        double error = 0;
        for (int i = 0; i < X.size(); ++i) {
            double e = X[i] - setPoints[i];
            error += e * e;
        }
        return sqrt(error);
    }

    void DroneController::ekf_subscriber(const nav_msgs::Odometry_<std::allocator<void>>::ConstPtr &msg) {
        auto position = msg->pose.pose.position;

        auto pose = msg->pose.pose;

        auto q = tf::Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
        double roll, pitch, yaw;
        tf::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);

        if(state_.empty())
        {
            state_.resize(4);
        }

        state_[0] = position.x;
        state_[1] = position.y;
        state_[2] = position.z;
        state_[3] = yaw;

    }


} // bebop2