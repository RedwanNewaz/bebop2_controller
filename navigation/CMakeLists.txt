
include_directories(trajectory)

set(navigation_HEADER_FILES
    ${PROJECT_SOURCE_DIR}/navigation/waypoints/WaypointController.h
    ${PROJECT_SOURCE_DIR}/navigation/ptp_move/P2PNav.h
)

set(navigation_SRC_FILES
    ${PROJECT_SOURCE_DIR}/navigation/waypoints/WaypointController.cpp
    ${PROJECT_SOURCE_DIR}/navigation/ptp_move/P2PNav.cpp
)

add_executable(p2p_navigation_server interface/p2p_navigation_server.cpp
        ${navigation_HEADER_FILES} ${navigation_SRC_FILES})

add_dependencies(p2p_navigation_server bebop2_controller_generate_messages_cpp)

target_link_libraries(
        p2p_navigation_server
        ${catkin_LIBRARIES}
        ${Boost_LIBRARIES}
)

add_executable(waypoint_control_server interface/waypoint_control_server.cpp
        ${navigation_HEADER_FILES} ${navigation_SRC_FILES}
)


add_dependencies(waypoint_control_server p2p_navigation_server)

target_link_libraries(
        waypoint_control_server
        ${catkin_LIBRARIES}
        ${Boost_LIBRARIES}
)

add_library(airlib_nav ${navigation_HEADER_FILES} ${navigation_SRC_FILES})
target_link_libraries(airlib_nav
        ${catkin_LIBRARIES}
        Eigen3::Eigen
        -pthread)
