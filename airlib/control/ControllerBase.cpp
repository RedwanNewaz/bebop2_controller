//
// Created by redwan on 12/15/22.
//

#include <numeric>
#include <utility>
#include "airlib/control/ControllerBase.h"

namespace bebop2
{
    
    ControllerBase::ControllerBase(StateObserverPtr  mGetState, ros::NodeHandle& nh) :
    m_get_state(mGetState), m_nh(nh) {

        ros::param::get("~dt", dt_);
        ros::param::get("~goal_thres", m_goal_thres);

        joystick_sub_ = m_nh.subscribe("/joy", 10, &bebop2::ControllerBase::joystick_callback, this);
        drone_takeoff_pub_ = m_nh.advertise<std_msgs::Empty>("takeoff", 1);
        drone_land_pub_ = m_nh.advertise<std_msgs::Empty>("land", 1);
        cmd_vel_pub_ = m_nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
        joystick_timer_ = m_nh.createTimer(ros::Duration(0.05), &bebop2::ControllerBase::joystick_timer_callback, this);
        controller_timer_ = m_nh.createTimer(ros::Duration(dt_), &bebop2::ControllerBase::control_loop, this);
        viz_ = std::make_unique<ControlViz>(m_nh);
        m_buttonState = ENGAGE;

    }

    
    void ControllerBase::joystick_callback(const sensor_msgs::Joy_<std::allocator<void>>::ConstPtr &msg) {

        std::lock_guard<std::mutex> lk(m_mu);
        auto it = std::find(msg->buttons.begin(), msg->buttons.end(), 1);
        if(it != msg->buttons.end())
        {
            int index = std::distance(msg->buttons.begin(), it);
            m_buttonState = static_cast<ButtonState>(index);

            std_msgs::Empty empty;

            if(m_buttonState == TAKEOFF)
            {
                drone_takeoff_pub_.publish(empty);
                ROS_INFO("[Button] pressed = %d -> TAKE OFF", m_buttonState);
                m_buttonState = ENGAGE;
            }

            if(m_buttonState == LAND)
            {
                drone_land_pub_.publish(empty);
                ROS_INFO("[Button] pressed = %d -> LAND", m_buttonState);
            }


            if(!setPoints_.empty())
            {
                auto pose = getStateVecToTransform(setPoints_);
                switch (m_buttonState) {
                    case TAKEOFF: viz_->update(pose, GREEN); break;
                    case IDLE:    viz_->update(pose, BLUE); break;
                    case ENGAGE:  viz_->update(pose, YELLOW);  break;
                    case CONTROL: viz_->update(pose, CYAN);   break;
                    case LAND:    viz_->update(pose, RED);    break;
                }
            }

        }
        axes_values_.clear();
        std::copy(msg->axes.begin(), msg->axes.end(), std::back_inserter(axes_values_));

    }

    
    void ControllerBase::joystick_timer_callback(const ros::TimerEvent &event) {
        if(axes_values_.empty() || std::accumulate(axes_values_.begin(), axes_values_.end(), 0.0) == 0.0)
            return;

        auto filter = [&](float val){
            float sign = val >= 0 ? 1 : -1;
            return abs(val) > DEAD_ZONE ? sign * STEP_INCR : 0;
        };

        float dz = filter(axes_values_[Z_AXIS_INDEX]);
        float dx = filter(axes_values_[X_AXIS_INDEX]);
        float dy = filter(axes_values_[Y_AXIS_INDEX]);

        //  update_setpoint with (dx, dy, dz);
        if(!setPoints_.empty())
        {   setPoints_[0] += dx;
            setPoints_[1] += dy;
            setPoints_[2] += dz;

            auto pose = getStateVecToTransform(setPoints_);
            switch (m_buttonState) {
                case TAKEOFF: viz_->update(pose, GREEN); break;
                case IDLE:    viz_->update(pose, BLUE); break;
                case ENGAGE:  viz_->update(pose, YELLOW);  break;
                case CONTROL: viz_->update(pose, CYAN);   break;
                case LAND:    viz_->update(pose, RED);    break;
            }
        }

    }




    
    void ControllerBase::control_loop(const ros::TimerEvent &event) {
        std::vector<double> state;
        m_get_state->operator()(state);
//        ROS_INFO_STREAM("[ControllerBase] state size" << state.size());
        if(state.empty())
            return;

        if(m_buttonState == ENGAGE)
        {
            // set setpoint at current position
            setPoints_.clear();
            std::copy(state.begin(), state.end(), std::back_inserter(setPoints_));

        }
        else if(m_buttonState == CONTROL)
        {
            if(goal_distance(state, setPoints_) > m_goal_thres)
            {
                // actively control position
                std::vector<double>U(NUM_CONTROLS);
                compute_control(state, setPoints_, U);
                ROS_INFO("[PositionController] vx = %lf, vy = %lf, vz = %lf", U[0], U[1], U[2]);
                publish_cmd_vel(U);
            }

        }

        // show drone
        auto transform = getStateVecToTransform(state);
        viz_->setDrone(transform);

    }


    
    void ControllerBase::publish_cmd_vel(const std::vector<double> &U) {

        std::lock_guard<std::mutex> lk(m_mu);
        assert(U.size() == 4 && "4 commands must be sent");
        geometry_msgs::Twist msg;

        msg.linear.x  = U[0];
        msg.linear.y  = U[1];
        msg.linear.z  = U[2];
        msg.angular.z = U[3];
        cmd_vel_pub_.publish(msg);

    }

    
    double ControllerBase::goal_distance(const std::vector<double> &X, const std::vector<double> &setPoints) {
        double error = 0;
        for (int i = 0; i < X.size(); ++i) {
            double e = X[i] - setPoints[i];
            error += e * e;
        }
        return sqrt(error);
    }


    
    tf::Transform ControllerBase::getStateVecToTransform(const std::vector<double> &state) {
        tf::Transform pose;
        tf::Quaternion q;
        q.setRPY(0, 0, state[3]);
        pose.setOrigin(tf::Vector3(state[0], state[1], state[2]));
        pose.setRotation(q);
        return pose;
    }


}