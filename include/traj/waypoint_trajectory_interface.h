//
// Created by redwan on 9/30/23.
//

#ifndef BEBOP2_CONTROLLER_WAYPOINT_TRAJECTORY_INTERFACE_H
#define BEBOP2_CONTROLLER_WAYPOINT_TRAJECTORY_INTERFACE_H
#include <Eigen/Dense>
#include <utility>
#include "message_queue.h"

typedef std::vector<std::vector<double>> WAYPOINTS;

class waypoint_trajectory_interface
{
    using TRAJECTORY = std::vector<std::vector<double>>;
public:
    waypoint_trajectory_interface(double max_vel, double max_acc, std::shared_ptr<MessageQueue> msg)
    :max_acc_(max_acc), max_vel_(max_vel), msg_(std::move(msg))
    {
        this->wp_size_ = 0;
    }
    ~waypoint_trajectory_interface() {

        // Wait for the worker thread to finish
        if (worker_thread_.joinable()) {
            worker_thread_.join();
        }

        msg_->setQuit(true);
//        std::cout <<"thread terminated " << std::endl;
    }

    virtual void convert_waypoints(const WAYPOINTS& wp)
    {
        int count = 0;
        wp_size_ = wp.size() + (wp.size() % 2);
        waypoints_.resize(3, wp_size_);

        for (int j = 0; j < wp_size_; ++j)
        {
            waypoints_(0, count ) = wp[j % wp.size()][0];
            waypoints_(1, count ) = wp[j % wp.size()][1];
            waypoints_(2, count ) = wp[j % wp.size()][2];
            count++;
        }
    }

    void start(const WAYPOINTS& wp)
    {
        convert_waypoints(wp);
        worker_thread_ = std::thread(&waypoint_trajectory_interface::run, this);
    }

    size_t size()
    {
        return wp_size_;
    }

protected:
    Eigen::MatrixXd waypoints_;
    double max_vel_, max_acc_;
    size_t wp_size_;
    std::thread worker_thread_;
    std::shared_ptr<MessageQueue> msg_;
    TRAJECTORY traj_;


protected:


    void run()
    {

        double start_time = traj_[0][0];

        for (int k = 0; k < traj_.size(); ++k)
        {
            //let action server know that it is last element in the trajectory
            // action server then change msg frame_id which will affect controller how to
            // reason about the goal threshold
            msg_->setQuit(k == (traj_.size() - 1));
            int nap_time = (traj_[k][0] - start_time) * 1000;
            std::this_thread::sleep_for(std::chrono::milliseconds(nap_time));

            std::vector<double>x{traj_[k][1], traj_[k][2], traj_[k][3], M_PI_2};
            //interface.set_goal_state(x);
            if(msg_->isTerminated())
                break;
            while (msg_->isPaused())
                std::this_thread::sleep_for(std::chrono::milliseconds(nap_time));
            // Set the message using the promise
            msg_->push(x);

//            std::cout << x[0] << ", " << x[1] << std::endl;

            start_time = traj_[k][0];
        }
        msg_->setTerminate(true);

        std::cout <<"execution terminated " << std::endl;

    }

    template<class T>
    TRAJECTORY path_to_trajectory(const T& raw_trajectory)
    {
        //            populate trajectory
        TRAJECTORY trajectory;
        float total_time = 0;
        for(auto traj:raw_trajectory)
        {
            float time = 0;
            while( time <= traj.getDuration())
            {
                auto pos = traj.getPos(time);
                trajectory.push_back(std::vector<double>{total_time, pos[0], pos[1], pos[2]});
                time += 0.1;
                total_time += 0.1;
            }
        }
        return trajectory;

    }

    Eigen::VectorXd allocateTime(const Eigen::MatrixXd &wayPs,
                          double vel,
                          double acc)
    {
        int N = (int)(wayPs.cols()) - 1;
        Eigen::VectorXd durations(N);
        if (N > 0)
        {

            Eigen::Vector3d p0, p1;
            double dtxyz, D, acct, accd, dcct, dccd, t1, t2, t3;
            for (int k = 0; k < N; k++)
            {
                p0 = wayPs.col(k);
                p1 = wayPs.col(k + 1);
                D = (p1 - p0).norm();

                acct = vel / acc;
                accd = (acc * acct * acct / 2);
                dcct = vel / acc;
                dccd = acc * dcct * dcct / 2;

                if (D < accd + dccd)
                {
                    t1 = sqrt(acc * D) / acc;
                    t2 = (acc * t1) / acc;
                    dtxyz = t1 + t2;
                }
                else
                {
                    t1 = acct;
                    t2 = (D - accd - dccd) / vel;
                    t3 = dcct;
                    dtxyz = t1 + t2 + t3;
                }

                durations(k) = dtxyz;
            }
        }

        return durations;
    }

    struct point {
        double x;
        double y;
        double z;

        double operator -(const point&other) const
        {
            double dx = this->x - other.x;
            double dy = this->y - other.y;

            return std::sqrt(dx * dx + dy * dy);
        }
    };

    // interpolation
    WAYPOINTS interpolateWaypoints(const std::vector<point>& waypoints, double resolution) {
        WAYPOINTS interpolatedWaypoints;

        // Iterate through each pair of waypoints
        for (size_t i = 0; i < waypoints.size() - 1; ++i) {
            const point& start = waypoints[i];
            const point& end = waypoints[i + 1];

            // Calculate the distance between start and end waypoints
            double dx = end.x - start.x;
            double dy = end.y - start.y;
            double dz = end.z - start.z;
            double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

            // Calculate the number of interpolated points between start and end waypoints
            int numInterpolatedPoints = std::ceil(distance / resolution);

            // Calculate the step size for interpolation
            double stepSize = 1.0 / numInterpolatedPoints;

            // Interpolate and add the waypoints
            for (int j = 0; j <= numInterpolatedPoints; ++j) {
                double t = stepSize * j;
                double interpolatedX = start.x + t * dx;
                double interpolatedY = start.y + t * dy;
                double interpolatedZ = start.z + t * dz;
                interpolatedWaypoints.push_back({interpolatedX, interpolatedY, interpolatedZ});
            }
        }

        return interpolatedWaypoints;
    }



};

#endif //BEBOP2_CONTROLLER_WAYPOINT_TRAJECTORY_INTERFACE_H
