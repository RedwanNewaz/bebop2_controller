//
// Created by redwan on 9/30/23.
//
#include <cmath>
#include "helper.h"
#include "catch2-2.7.0/catch.hpp"
#include "airlib/control/controller.h"




TEST_CASE("quad_pids::compute_control::north", "[move::north]")
{
    std::vector<double>gains{
            0.08, 0.20, 0,
            0.08, 0.20, 0,
            0.08, 0.20, 0,
            0.08, 0.20, 0
    };
    controller::quad_pids quadPids(gains, 0.3);

    std::vector<double> X0{0, 0, 0, M_PI_2};
    std::vector<double> Xg{0, 1, 0, M_PI_2};
    std::vector<double> result;

    quadPids.compute_control(X0, Xg, result);

    std::vector<double> expected{0.7466666667, 0.0, 0.0, 0.0 };

    double epsilon = 0.001;

    REQUIRE(areArraysEqualWithTolerance(expected, result, epsilon));

}



TEST_CASE("quad_pids::compute_control::south", "[move::south]")
{
    std::vector<double>gains{
            0.08, 0.20, 0,
            0.08, 0.20, 0,
            0.08, 0.20, 0,
            0.08, 0.20, 0
    };
    controller::quad_pids quadPids(gains, 0.3);

    std::vector<double> X0{0, 0, 0, M_PI_2};
    std::vector<double> Xg{0, -1, 0, M_PI_2};
    std::vector<double> result;

    quadPids.compute_control(X0, Xg, result);

    std::vector<double> expected{-0.7466666667, 0.0, 0.0, 0.0 };

    double epsilon = 0.001;
    REQUIRE(areArraysEqualWithTolerance(expected, result, epsilon));

}


TEST_CASE("quad_pids::compute_control::east", "[move::east]")
{
    std::vector<double>gains{
            0.08, 0.20, 0,
            0.08, 0.20, 0,
            0.08, 0.20, 0,
            0.08, 0.20, 0
    };
    controller::quad_pids quadPids(gains, 0.3);

    std::vector<double> X0{0, 0, 0, M_PI_2};
    std::vector<double> Xg{1, 0, 0, M_PI_2};
    std::vector<double> result;

    quadPids.compute_control(X0, Xg, result);

    std::vector<double> expected{0.0, -0.7466666667, 0.0, 0.0 };

    double epsilon = 0.001;
    REQUIRE(areArraysEqualWithTolerance(expected, result, epsilon));

}



TEST_CASE("quad_pids::compute_control::west", "[move::west]")
{
    std::vector<double>gains{
            0.08, 0.20, 0,
            0.08, 0.20, 0,
            0.08, 0.20, 0,
            0.08, 0.20, 0
    };
    controller::quad_pids quadPids(gains, 0.3);

    std::vector<double> X0{0, 0, 0, M_PI_2};
    std::vector<double> Xg{-1, 0, 0, M_PI_2};
    std::vector<double> result;

    quadPids.compute_control(X0, Xg, result);

    std::vector<double> expected{0.0, 0.7466666667, 0.0, 0.0 };

    double epsilon = 0.001;
    
    REQUIRE(areArraysEqualWithTolerance(expected, result, epsilon));

}

TEST_CASE("quad_pids::compute_control::ccw", "[rotate::ccw]")
{
    std::vector<double>gains{
            0.08, 0.20, 0,
            0.08, 0.20, 0,
            0.08, 0.20, 0,
            0.08, 0.20, 0
    };
    controller::quad_pids quadPids(gains, 0.3);

    std::vector<double> X0{0, 0, 0, M_PI_2};
    std::vector<double> Xg{0, 0, 0, M_PI_2 + M_PI_4};
    std::vector<double> result;

    quadPids.compute_control(X0, Xg, result);

    std::vector<double> expected{0.0, 0.0, 0.0, -0.5864306287 };

    double epsilon = 0.001;
    REQUIRE(areArraysEqualWithTolerance(expected, result, epsilon));

}

TEST_CASE("quad_pids::compute_control::cw", "[rotate::cw]")
{
    std::vector<double>gains{
            0.08, 0.20, 0,
            0.08, 0.20, 0,
            0.08, 0.20, 0,
            0.08, 0.20, 0
    };
    controller::quad_pids quadPids(gains, 0.3);

    std::vector<double> X0{0, 0, 0, M_PI_2};
    std::vector<double> Xg{0, 0, 0, M_PI_2 - M_PI_4};
    std::vector<double> result;

    quadPids.compute_control(X0, Xg, result);

    std::vector<double> expected{0.0, 0.0, 0.0, 0.5864306287 };

    double epsilon = 0.001;
    REQUIRE(areArraysEqualWithTolerance(expected, result, epsilon));

}