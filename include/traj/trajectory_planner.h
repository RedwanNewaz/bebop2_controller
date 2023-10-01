//
// Created by redwan on 10/1/23.
//

#ifndef BEBOP2_CONTROLLER_TRAJECTORY_PLANNER_H
#define BEBOP2_CONTROLLER_TRAJECTORY_PLANNER_H
#include <utility>
#include "traj_min_jerk.hpp"
#include "traj_min_snap.hpp"
#include "waypoint_trajectory_interface.h"

namespace traj_planner
{

    class constant_velocity: public waypoint_trajectory_interface
    {
    public:
        constant_velocity(double max_vel, double max_acc, std::shared_ptr<MessageQueue> msg)
                :waypoint_trajectory_interface(max_vel, max_acc, std::move(msg))
        {
            this->wp_size_ = 0;
        }

        void convert_waypoints(const WAYPOINTS& wp) override
        {
            //convert it to a path: vector of waypoints
            size_t N = wp.size();
            std::vector<point> path_points(N);
            for (int i = 0; i < N; ++i)
                path_points[i] = point{wp[i][0], wp[i][1], wp[i][2]};

            // interpolate the path based on a fixed distance
            double m_resolution = 0.1;
            auto final_path = interpolateWaypoints(path_points, m_resolution);
            this->wp_size_ = final_path.size();

            traj_.clear();
            double time = 0;
            double dt =  max_vel_/ max_acc_;
            for (auto & i : final_path) {
                traj_.push_back({time, i[0], i[1], i[2]});
                // printf("[CV %d] (%lf, %lf, %lf, %lf)\n", i, time, final_path[i][0], final_path[i][1], final_path[i][2]);
                time += dt;
            }
        }
    };

    class minimum_snap: public waypoint_trajectory_interface
    {
    public:
        minimum_snap(double max_vel, double max_acc, std::shared_ptr<MessageQueue> msg)
                :waypoint_trajectory_interface(max_vel, max_acc, std::move(msg))
        {
            this->wp_size_ = 0;
        }


        void convert_waypoints(const WAYPOINTS& wp) override
        {
            waypoint_trajectory_interface::convert_waypoints(wp);
            Eigen::VectorXd ts;
            Eigen::Matrix3d iS, fS;
            Eigen::Matrix<double, 3, 4> iSS, fSS;

            min_snap::SnapOpt snapOpt;
            min_snap::Trajectory minSnapTraj;
            iS.setZero();
            fS.setZero();

            iS.col(0) << waypoints_.leftCols<1>();
            fS.col(0) << waypoints_.rightCols<1>();
            ts = allocateTime(waypoints_, max_vel_, max_acc_);

            iSS << iS, Eigen::MatrixXd::Zero(3, 1);
            fSS << fS, Eigen::MatrixXd::Zero(3, 1);

            int pieceNum = waypoints_.cols() - 1;
            snapOpt.reset(iSS, fSS, pieceNum);

            snapOpt.generate(waypoints_.block(0, 1, 3, pieceNum - 1), ts);
            snapOpt.getTraj(minSnapTraj);

            traj_ = path_to_trajectory(minSnapTraj);
            wp_size_ =  traj_.size();
        }

    };

    class minimum_jerk: public waypoint_trajectory_interface
    {
    public:
        minimum_jerk(double max_vel, double max_acc, std::shared_ptr<MessageQueue> msg)
                :waypoint_trajectory_interface(max_vel, max_acc, std::move(msg))
        {
            this->wp_size_ = 0;
        }


        void convert_waypoints(const WAYPOINTS& wp) override
        {
            waypoint_trajectory_interface::convert_waypoints(wp);
            Eigen::VectorXd ts;
            Eigen::Matrix3d iS, fS;

            min_jerk::JerkOpt jerkOpt;
            min_jerk::Trajectory minJerkTraj;

            iS.setZero();
            fS.setZero();

            iS.col(0) << waypoints_.leftCols<1>();
            fS.col(0) << waypoints_.rightCols<1>();
            ts = allocateTime(waypoints_, max_vel_, max_acc_);

            int pieceNum = waypoints_.cols() - 1;
            jerkOpt.reset(iS, fS, pieceNum);

            jerkOpt.generate(waypoints_.block(0, 1, 3, pieceNum - 1), ts);
            jerkOpt.getTraj(minJerkTraj);

            traj_ = path_to_trajectory(minJerkTraj);
            wp_size_ =  traj_.size();
        }

    };
}

#endif //BEBOP2_CONTROLLER_TRAJECTORY_PLANNER_H
