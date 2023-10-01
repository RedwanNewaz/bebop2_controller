//
// Created by redwan on 9/30/23.
//
#include <cmath>
#include "helper.h"
#include "catch2-2.7.0/catch.hpp"
#include "airlib/control/controller.h"
#include "trajectory_planner.h"

TEST_CASE("traj_planner::constant_velocity::size", "[wp::rectangle]")
{
    WAYPOINTS  wps{
        {0.0, 0.0, 1.2},
        {0.0, 6.0, 1.2},
        {6.0, 6.0, 1.2},
        {6.0, 0.0, 1.2}
    };

    auto messageQueue = std::make_shared<MessageQueue>();
    traj_planner::constant_velocity communicator(2, 3, messageQueue);
    communicator.convert_waypoints(wps);
    size_t expected = 183;
    REQUIRE(communicator.size() == expected);

}

TEST_CASE("traj_planner::minimum_snap::size", "[wp::rectangle]")
{
    WAYPOINTS  wps{
            {0.0, 0.0, 1.2},
            {0.0, 6.0, 1.2},
            {6.0, 6.0, 1.2},
            {6.0, 0.0, 1.2}
    };

    auto messageQueue = std::make_shared<MessageQueue>();
    traj_planner::minimum_snap communicator(2, 3, messageQueue);
    communicator.convert_waypoints(wps);
    size_t expected = 111;
    REQUIRE(communicator.size() == expected);

}

TEST_CASE("traj_planner::minimum_jerk::size", "[wp::rectangle]")
{
    WAYPOINTS  wps{
            {0.0, 0.0, 1.2},
            {0.0, 6.0, 1.2},
            {6.0, 6.0, 1.2},
            {6.0, 0.0, 1.2}
    };

    auto messageQueue = std::make_shared<MessageQueue>();
    traj_planner::minimum_jerk communicator(2, 3, messageQueue);
    communicator.convert_waypoints(wps);
    size_t expected = 111;
    REQUIRE(communicator.size() == expected);

}