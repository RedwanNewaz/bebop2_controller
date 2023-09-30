#include "airlib/localization/Sensors/DummyState.h"


namespace bebop2
{
    DummyState::DummyState(const std::vector<double>& noise):noise_(noise)
    {
        states_.resize(STATE_DIM);
        std::fill(states_.begin(), states_.end(), 0.0);
//        states_[STATE_DIM - 1] = M_PI_2;

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

        double theta = states_[3] + M_PI_2;

        double c = cos(theta);
        double s = sin(theta);

        Eigen::Vector3d p;

        // parrot bebop2 moves left for (+) linear.y and vice versa
        double factor = 1.0;
        p <<  msg->linear.x * factor ,
              msg->linear.y * factor ,
              msg->linear.z * factor ;
        states_[3] -= msg->angular.z * factor;

        Eigen::Matrix3d q;
        q.setIdentity();
        q(0, 0) = c;
        q(0, 1) = -s;
        q(1, 0) = s;
        q(1, 1) = c;

        Eigen::Vector3d T = q * p ;

        states_[0] += T(0);
        states_[1] += T(1);
        states_[2] += T(2);
        states_[3] = (fmod(states_[3] + M_PI, 2 * M_PI) - M_PI);




    }



} // namespace bebop2

