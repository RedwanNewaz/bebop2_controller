#include "airlib/localization/Sensors/DummyState.h"


namespace bebop2
{
    DummyState::DummyState(ros::NodeHandle& nh, bool noisy_reading):nh_(nh), noisy_reading_(noisy_reading)
    {
        states_.resize(STATE_DIM);
        std::fill(states_.begin(), states_.end(), 0.0);

        sub_takeoff_ = nh_.subscribe("takeoff", 10, &DummyState::takeoff_callback, this);
        sub_land_ = nh_.subscribe("land", 10, &DummyState::land_callback, this);
        sub_cmd_vel_ = nh_.subscribe("cmd_vel", 1, &DummyState::cmd_vel_callback, this);
    }

    void DummyState::operator()(std::vector<double>& state)
    {
        state.clear();
        std::copy(states_.begin(), states_.end(), std::back_inserter(state));

//        ROS_INFO("state = (%lf %lf %lf %lf)", states_[0], states_[1], states_[2], states_[3] );

        // don't add noise if noisy_reading is not enabled
        if(!noisy_reading_)
            return;

        // add gaussian white noise for simulating a realistic measurement
        std::normal_distribution<double> disturbance(0.0, sqrt(NOISE));
        //focus on x, y coordinates only. Later tune it for all 4 axes
        for(std::size_t i=0; i<state.size() - 2; ++i)
            state[i] += disturbance(rd_);

    }

    bool DummyState::empty()
    {
        return states_.empty();
    }

    void DummyState::takeoff_callback(const std_msgs::Empty::ConstPtr& msg)
    {
        states_[2] = 1.0;
        ROS_INFO("[DummyState] takeoff message received");
    }
    void DummyState::land_callback(const std_msgs::Empty::ConstPtr& msg)
    {
        states_[2] = 0.0;
        ROS_INFO("[DummyState] land message received");
    }
    void DummyState::cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg)
    {
        states_[0] += DT * msg->linear.x;
        // parrot bebop2 moves left for (+) linear.y and vice versa
        states_[1] -= DT * msg->linear.y;
        states_[2] += DT * msg->linear.z;
        states_[3] += DT * msg->angular.z;

        ROS_INFO("[DummyState] state = (%lf, %lf, %lf, %lf)", states_[0], states_[1], states_[2], states_[3]);
    }
    
} // namespace bebop2

