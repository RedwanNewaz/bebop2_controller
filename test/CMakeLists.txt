include_directories(../include/catch2-2.7.0)
add_library(Catch INTERFACE)
add_executable( bebop2_controller_test
        test_apriltag_landmark.cpp
        test.cpp
        test_quad_pids.cpp
        helper.h
        test_waypoint_trajectory.cpp

        ${perception_HEADER_FILES} ${perception_SRC_FILES}
        ${navigation_HEADER_FILES} ${navigation_SRC_FILES}
        ${controller_HEADER_FILES} ${controller_SRC_FILES}
)
target_link_libraries(bebop2_controller_test Catch Eigen3::Eigen
        ${catkin_LIBRARIES}
        airlib_percept
        airlib_nav
        airlib_control
        -pthread)

enable_testing()
add_test( NAME bebop2_controller_test COMMAND bebop2_controller_test )