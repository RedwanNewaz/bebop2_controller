//
// Created by redwan on 9/30/23.
//

#ifndef BEBOP2_CONTROLLER_WAYPOINT_TRAJECTORY_H
#define BEBOP2_CONTROLLER_WAYPOINT_TRAJECTORY_H
#include <Eigen/Dense>
#include "traj_min_jerk.hpp"
#include "traj_min_snap.hpp"
#include "message_queue.h"

typedef std::vector<std::vector<double>> WAYPOINTS;

class waypoint_trajectory
{
    using TRAJECTORY = std::vector<std::vector<double>>;
public:
    waypoint_trajectory(double max_vel, double max_acc, std::shared_ptr<MessageQueue> msg)
    :max_acc_(max_acc), max_vel_(max_vel), msg_(msg)
    {

    }
    ~waypoint_trajectory() {

        // Wait for the worker thread to finish
        if (worker_thread_.joinable()) {
            worker_thread_.join();
        }
    }

    void start(const WAYPOINTS& wp)
    {
        convert_waypoints(wp);
        worker_thread_ = std::thread(&waypoint_trajectory::run, this);
    }

private:
    Eigen::MatrixXd waypoints_;
    double max_vel_, max_acc_;
    int wp_size_;
    std::thread worker_thread_;
    std::shared_ptr<MessageQueue> msg_;


private:
    void convert_waypoints(const WAYPOINTS& wp)
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

    void run()
    {
//        auto trajectory = path_to_trajectory(waypoints, N - 1, max_vel, max_acc);
        auto trajectory = path_to_trajectory(waypoints_, wp_size_ - 1, max_vel_, max_acc_);

        double start_time = trajectory[0][0];

        for (int k = 0; k < trajectory.size(); ++k)
        {
            int nap_time = (trajectory[k][0] - start_time) * 1000;
            std::this_thread::sleep_for(std::chrono::milliseconds(nap_time));

            std::vector<double>x{trajectory[k][1], trajectory[k][2], trajectory[k][3], M_PI_2};
            //interface.set_goal_state(x);
            // Set the message using the promise
            msg_->push(x);

            std::cout << x[0] << ", " << x[1] << std::endl;

            start_time = trajectory[k][0];
        }
    }

    TRAJECTORY path_to_trajectory(const Eigen::MatrixXd& waypoints, int num_points, double max_vel, double max_acc)
    {
        Eigen::VectorXd ts;
        Eigen::Matrix3d iS, fS;
        Eigen::Matrix<double, 3, 4> iSS, fSS;

        min_snap::SnapOpt snapOpt;
        min_snap::Trajectory minSnapTraj;
        iS.setZero();
        fS.setZero();

        iS.col(0) << waypoints.leftCols<1>();
        fS.col(0) << waypoints.rightCols<1>();
        ts = allocateTime(waypoints, max_vel, max_acc);

        iSS << iS, Eigen::MatrixXd::Zero(3, 1);
        fSS << fS, Eigen::MatrixXd::Zero(3, 1);

        snapOpt.reset(iSS, fSS, waypoints.cols() - 1);
        snapOpt.generate(waypoints.block(0, 1, 3, num_points - 1), ts);
        snapOpt.getTraj(minSnapTraj);



        //            populate trajectory
        TRAJECTORY trajectory;
        float total_time = 0;
        for(auto traj:minSnapTraj)
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



};

#endif //BEBOP2_CONTROLLER_WAYPOINT_TRAJECTORY_H
