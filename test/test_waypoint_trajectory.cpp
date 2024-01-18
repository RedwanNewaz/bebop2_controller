//
// Created by redwan on 9/30/23.
//
#include <cmath>
#include "helper.h"
#include "catch2-2.7.0/catch.hpp"
#include "../navigation/trajectory/rapidcsv.h"
#include "../navigation/trajectory/trajectory_planner.h"
#include "LoggerCSV.h"
#include <filesystem>

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
    size_t expected = 123;
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
    int numPoints = 300;
    double xScale = 1.5;
    double yScale = 1.5;
    WAYPOINTS wps;

    for (double i = 0; i < M_PI * 2; i+= 0.1) {
        double a = 0.5;
        double Y =  5.2 + 2 * a * (1 - cos(i)) * cos(i); // You can adjust the scaling factor (2) for size
        double X =  2.5 + 2 * a * (1 - cos(i)) * sin(i);; // You can adjust the vertical offset (+1) for position
        wps.push_back({X, Y, 1.2});
    }
    return wps;
}

WAYPOINTS calcEight() {
//    // Define a parameter t
    int numPoints = 50;
    double xScale = 2.4;
    double yScale = 1.8;
    WAYPOINTS wps;

    for (int i = 0; i < numPoints; ++i) {
        double t = 2 * M_PI * i / numPoints;
        double xx = 2.9 + xScale * cos(t) * sin(t); // You can adjust the scaling factor (2) for size
        double yy = 3.8 + yScale * sin(t); // You can adjust the vertical offset (+1) for position
        wps.push_back({xx, yy, 1.0});
    }

    return wps;
}
WAYPOINTS calcSquare() {
    int numPoints = 4; // For a square, you need only 4 points
    double sideLength = 2.0; // Adjust the side length as needed
    WAYPOINTS wps;

    for (int i = 0; i < numPoints; ++i) {
        double angle = 2 * M_PI * i / numPoints;
        double x = 1.5 + sideLength * cos(angle); // Adjust the horizontal offset (2.9) as needed
        double y = 3 + sideLength * sin(angle); // Adjust the vertical offset (3.8) as needed
        wps.push_back({x, y, 1.0});
    }

    return wps;
}



TEST_CASE("traj_planner::eight_path", "[wp::eight]")
{
    WAYPOINTS  wps = calcSquare();

    std::string output = "/home/roboticslab/catkin_ws/src/bebop2_controller/test";
    LoggerCSV loggerCsv;
    loggerCsv.setOutputFolder(output);


    for(const auto& row: wps)
    {
        std::vector<std::string> data;
        for(const auto& item : row)
            data.emplace_back(std::to_string(item));
        loggerCsv.addRow(data);
    }

    bool isValidoutput = std::filesystem::is_directory(output);

    REQUIRE_FALSE(!isValidoutput);
}