//
// Created by redwan on 9/30/23.
//
#include <cmath>
#include "helper.h"
#include "catch2-2.7.0/catch.hpp"
#include "airlib/control/controller.h"
#include "waypoint_trajectory.h"

TEST_CASE("waypoint_trajectory::size", "[wp::rectangle]")
{
    WAYPOINTS  wps{
        {0.0, 0.0, 1.2},
        {0.0, 6.0, 1.2},
        {6.0, 6.0, 1.2},
        {6.0, 0.0, 1.2}
    };

    auto messageQueue = std::make_shared<MessageQueue>();
    waypoint_trajectory communicator(2, 3, messageQueue);
    communicator.start(wps);
    std::this_thread::sleep_for(1s);
    messageQueue->setTerminate(true);
    size_t expected = 111;
    REQUIRE(communicator.size() == expected);

}