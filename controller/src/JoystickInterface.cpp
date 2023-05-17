//
// Created by airlab on 5/17/23.
//

#include "JoystickInterface.h"

namespace bebop2 {
    JoystickInterface::JoystickInterface() {
        joystick_sub_ = nh_.subscribe("/joy", 10, &bebop2::JoystickInterface::joystick_callback, this);
        drone_takeoff_pub_ = nh_.advertise<std_msgs::Empty>("/bebop/takeoff", 1);
        drone_land_pub_ = nh_.advertise<std_msgs::Empty>("/bebop/land", 1);
        joystick_timer_ = nh_.createTimer(ros::Duration(0.05), &bebop2::JoystickInterface::joystick_timer_callback, this);
        viz_ = std::make_unique<StateViz>();
        buttonState_ = ENGAGE;
        state_sub_ = nh_.subscribe("/bebop/ekf_state", 10, &JoystickInterface::ekf_subscriber, this);

        ROS_INFO("[JoystickInterface] Initialization complete ...");
    }

    std::vector<double> JoystickInterface::getSetpoint() const {
        return setPoints_;
    }

    ButtonState JoystickInterface::getButtonState() const {
        return buttonState_;
    }

    void JoystickInterface::joystick_callback(const sensor_msgs::Joy_<std::allocator<void>>::ConstPtr &msg) {
        std::lock_guard<std::mutex> lk(mu_);
        auto it = std::find(msg->buttons.begin(), msg->buttons.end(), 1);
        if(it != msg->buttons.end())
        {
            int index = std::distance(msg->buttons.begin(), it);
            buttonState_ = static_cast<ButtonState>(index);

            std_msgs::Empty empty;

            if(buttonState_ == TAKEOFF)
            {
                drone_takeoff_pub_.publish(empty);
                ROS_INFO("[Button] pressed = %d -> TAKE OFF", buttonState_);
                buttonState_ = ENGAGE;
            }

            if(buttonState_ == LAND)
            {
                drone_land_pub_.publish(empty);
                ROS_INFO("[Button] pressed = %d -> LAND", buttonState_);
            }


            if(!setPoints_.empty())
            {
                auto pose = getStateVecToTransform(setPoints_);
                switch (buttonState_) {
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

    tf::Transform JoystickInterface::getStateVecToTransform(const std::vector<double> &state) {
        tf::Transform pose;
        tf::Quaternion q;
        q.setRPY(0, 0, state[3]);
        pose.setOrigin(tf::Vector3(state[0], state[1], state[2]));
        pose.setRotation(q);
        return pose;
    }

    void JoystickInterface::joystick_timer_callback(const ros::TimerEvent &event) {

        bool needUpdate = !(axes_values_.empty() || std::accumulate(axes_values_.begin(), axes_values_.end(), 0.0) == 0.0);

        auto filter = [&](float val){
            float sign = val >= 0 ? 1 : -1;
            return abs(val) > DEAD_ZONE ? sign * STEP_INCR : 0;
        };

        float dx, dy, dz;
        dx = dy = dz = 0;

        if(needUpdate)
        {
            dz = filter(axes_values_[Z_AXIS_INDEX]);
            dx = filter(axes_values_[X_AXIS_INDEX]);
            dy = filter(axes_values_[Y_AXIS_INDEX]);
        }


        //  update_setpoint with (dx, dy, dz);
        if(!setPoints_.empty())
        {   setPoints_[0] += dx;
            setPoints_[1] += dy;
            setPoints_[2] += dz;

            auto pose = getStateVecToTransform(setPoints_);
            switch (buttonState_) {
                case TAKEOFF: viz_->update(pose, GREEN); break;
                case IDLE:    viz_->update(pose, BLUE); break;
                case ENGAGE:  viz_->update(pose, YELLOW);  break;
                case CONTROL: viz_->update(pose, CYAN);   break;
                case LAND:    viz_->update(pose, RED);    break;
            }
        }

    }


    void JoystickInterface::ekf_subscriber(const nav_msgs::Odometry::ConstPtr &msg) {

        if(buttonState_ == CONTROL)
            return;

        auto position = msg->pose.pose.position;

        if(setPoints_.empty())
        {
            setPoints_.push_back(position.x);
            setPoints_.push_back(position.y);
            setPoints_.push_back(position.z);
            setPoints_.push_back(0);
        }
        else if (setPoints_.size() >= 3)
        {
            setPoints_[0] = position.x;
            setPoints_[1] = position.y;
            setPoints_[2] = position.z;
        }


    }

} // bebop2