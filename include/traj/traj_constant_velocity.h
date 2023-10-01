//
// Created by redwan on 10/1/23.
//

#ifndef BEBOP2_CONTROLLER_TRAJ_CONSTANT_VELOCITY_H
#define BEBOP2_CONTROLLER_TRAJ_CONSTANT_VELOCITY_H
#include <utility>

#include "waypoint_trajectory.h"

class traj_constant_velocity: public waypoint_trajectory
{
public:
    traj_constant_velocity(double max_vel, double max_acc, std::shared_ptr<MessageQueue> msg)
            :waypoint_trajectory(max_vel, max_acc, std::move(msg))
    {
        this->wp_size_ = 0;
    }
protected:
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


        traj_.clear();
        double time = 0;
        double dt =  max_vel_/ max_acc_;
        for (int i = 0; i < final_path.size(); ++i) {
            traj_.push_back({time, final_path[i][0], final_path[i][1], final_path[i][2]});
//            printf("[CV %d] (%lf, %lf, %lf, %lf)\n", i, time, final_path[i][0], final_path[i][1], final_path[i][2]);

            time += dt;
        }

    }

private:
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
#endif //BEBOP2_CONTROLLER_TRAJ_CONSTANT_VELOCITY_H
