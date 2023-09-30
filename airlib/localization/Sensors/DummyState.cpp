#include "airlib/localization/Sensors/DummyState.h"


namespace bebop2
{
    DummyState::DummyState(const std::vector<double>& noise):noise_(noise)
    {
        states_.resize(STATE_DIM);
        std::fill(states_.begin(), states_.end(), 0.0);

    }
    void DummyState::set(std::vector<double>& state)
    {
        std::copy(state.begin(), state.end(), states_.begin());
    }

    void DummyState::operator()(std::vector<double>& state)
    {
        std::copy(states_.begin(), states_.end(), std::back_inserter(state));


        //focus on x, y coordinates only. Later tune it for all 4 axes
        for(std::size_t i=0; i<noise_.size(); ++i) {
            // add gaussian white noise for simulating a realistic measurement
            std::normal_distribution<double> disturbance(0.0, sqrt(noise_[i]));
            state[i] += disturbance(rd_);
        }
        last_update_ = std::chrono::high_resolution_clock::now();
    }

    bool DummyState::empty()
    {
        // Get the current time after the operation
        auto current = std::chrono::high_resolution_clock::now();
        // Calculate the elapsed time
        std::chrono::duration<double> elapsed_seconds = current - last_update_;

        return elapsed_seconds.count() < DT;
    }

    void DummyState::takeoff_callback()
    {
        states_[2] = 1.0;

    }
    void DummyState::land_callback()
    {
        states_[2] = 0.0;

    }
    void DummyState::cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg)
    {
        states_[1] += DT * msg->linear.x;
        // parrot bebop2 moves left for (+) linear.y and vice versa
        states_[0] -= DT * msg->linear.y;
        states_[2] += DT * msg->linear.z;
        states_[3] += DT * msg->angular.z;

    }



} // namespace bebop2

