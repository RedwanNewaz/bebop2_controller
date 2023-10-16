//
// Created by airlab on 10/16/23.
//
#include <cmath>
#include "helper.h"
#include "catch2-2.7.0/catch.hpp"
#include "airlib/controller.hpp"
std::vector<double>gains{
        0.075, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0,
        0.0, 0.075, 0.0, 0.0, 0.0, 0.00075, 0.0, 0.0,
        0.0, 0.0, 0.085, 0.0, 0.0, 0.0, 0.001, 0.0,
        0.0, 0.0, 0.0, 0.08, 0.0, 0.0, 0.0, 0.002,
        0.0, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.00075, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.001, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.002
};


TEST_CASE("quad_lqg::compute_control::north", "[move::north]")
{

    controller::quad_lqg controller(gains, 0.3);

    std::vector<double> X0{0, 0, 0, M_PI_2};
    std::vector<double> Xg{0, 1, 0, M_PI_2};
    std::vector<double> result;

    controller.compute_control(X0, Xg, result);

    std::vector<double> expected{0.103037, 0.0, 0.0, 0.0 };

    double epsilon = 0.001;

    REQUIRE(areArraysEqualWithTolerance(expected, result, epsilon));

}


TEST_CASE("quad_lqg::compute_control::south", "[move::south]")
{

    controller::quad_lqg controller(gains, 0.3);

    std::vector<double> X0{0, 0, 0, M_PI_2};
    std::vector<double> Xg{0, -1, 0, M_PI_2};
    std::vector<double> result;

    controller.compute_control(X0, Xg, result);

    std::vector<double> expected{-0.103037, 0.0, 0.0, 0.0 };

    double epsilon = 0.001;
    REQUIRE(areArraysEqualWithTolerance(expected, result, epsilon));

}


TEST_CASE("quad_lqg::compute_control::east", "[move::east]")
{
    std::vector<double> X0{0, 0, 0, M_PI_2};
    std::vector<double> Xg{1, 0, 0, M_PI_2};
    std::vector<double> result;

    controller::quad_lqg controller(gains, 0.3);
    controller.compute_control(X0, Xg, result);

    std::vector<double> expected{0.0, -0.119990, 0.0, 0.0 };

    double epsilon = 0.001;
    REQUIRE(areArraysEqualWithTolerance(expected, result, epsilon));

}



TEST_CASE("quad_lqg::compute_control::west", "[move::west]")
{
    std::vector<double> X0{0, 0, 0, M_PI_2};
    std::vector<double> Xg{-1, 0, 0, M_PI_2};
    std::vector<double> result;

    controller::quad_lqg controller(gains, 0.3);
    controller.compute_control(X0, Xg, result);

    std::vector<double> expected{0.0, 0.119990, 0.0, 0.0 };

    double epsilon = 0.001;

    REQUIRE(areArraysEqualWithTolerance(expected, result, epsilon));

}

TEST_CASE("quad_lqg::compute_control::ccw", "[rotate::ccw]")
{


    std::vector<double> X0{0, 0, 0, M_PI_2};
    std::vector<double> Xg{0, 0, 0, M_PI_2 + M_PI_4};
    std::vector<double> result;

    controller::quad_lqg controller(gains, 0.3);
    controller.compute_control(X0, Xg, result);

    std::vector<double> expected{0.0, 0.0, 0.0, 0.131224 };

    double epsilon = 0.001;
    REQUIRE(areArraysEqualWithTolerance(expected, result, epsilon));

}

TEST_CASE("quad_lqg::compute_control::cw", "[rotate::cw]")
{
    std::vector<double> X0{0, 0, 0, M_PI_2};
    std::vector<double> Xg{0, 0, 0, M_PI_2 - M_PI_4};
    std::vector<double> result;

    controller::quad_lqg controller(gains, 0.3);
    controller.compute_control(X0, Xg, result);

    std::vector<double> expected{0.0, 0.0, 0.0, -0.131224 };

    double epsilon = 0.001;
    REQUIRE(areArraysEqualWithTolerance(expected, result, epsilon));

}