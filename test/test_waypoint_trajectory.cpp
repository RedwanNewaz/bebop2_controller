//
// Created by redwan on 9/30/23.
//
#include <cmath>
#include "helper.h"
#include "catch2-2.7.0/catch.hpp"
#include "airlib/control/controller.h"
#include "trajectory_planner.h"
#include "airlib/utility/LoggerCSV.h"

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

WAYPOINTS calcInput() {
    // Define a parameter t
    int numPoints = 50;
    double xScale = 3.5;
    double yScale = 2.5;
    WAYPOINTS wps;

    for (int i = 0; i < numPoints; ++i) {
        double t = 2 * M_PI * i / numPoints;
        double xx = xScale + xScale * cos(t) * sin(t); // You can adjust the scaling factor (2) for size
        double yy = yScale + yScale * sin(t); // You can adjust the vertical offset (+1) for position
        wps.push_back({xx, yy, 1.2});
    }
    return wps;
}


TEST_CASE("traj_planner::eight_path", "[wp::eight]")
{
    WAYPOINTS  wps = calcInput();

//    std::string output = "/home/airlab/catkin_ws/src/bebop2_controller/test";
//    LoggerCSV loggerCsv;
//    loggerCsv.setOutputFolder(output);
//
//    for(const auto& row: wps)
//    {
//        std::vector<std::string> data;
//        for(const auto& item : row)
//            data.emplace_back(std::to_string(item));
//        loggerCsv.addRow(data);
//    }
    auto messageQueue = std::make_shared<MessageQueue>();
    traj_planner::minimum_jerk communicator(2, 3, messageQueue);
    communicator.convert_waypoints(wps);
    size_t expected = 363;
    REQUIRE(communicator.size() == expected);

}