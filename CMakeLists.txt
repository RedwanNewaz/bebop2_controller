cmake_minimum_required(VERSION 3.0.2)
cmake_policy(SET CMP0058 NEW)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-dev")

project(bebop2_controller)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_geometry
  image_transport
  nodelet
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  genmsg
  actionlib
  actionlib_msgs
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

add_action_files(
        DIRECTORY action
        FILES Waypoints.action Setpoints.action
)
generate_messages(
        DEPENDENCIES actionlib_msgs std_msgs  geometry_msgs # Or other packages containing msgs
)

catkin_package(
        CATKIN_DEPENDS actionlib_msgs geometry_msgs
#  INCLUDE_DIRS include
#  LIBRARIES airlib
#  CATKIN_DEPENDS cv_bridge geometry_msgs image_geometry image_transport nodelet roscpp rospy sensor_msgs std_msgs tf visp_bridge
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  include/traj
)

set(SOURCE_FILES
        airlib/localization/Filters/ekf/ExtendedKalmanFilter.cpp
        airlib/localization/StateObserver.cpp
        airlib/localization/Sensors/ApriltagLandmarks.cpp
        airlib/localization/Sensors/DummyState.cpp
        airlib/control/ControlViz.cpp
        airlib/control/PID/PID.cpp
        airlib/control/ControllerInterface.cpp
        airlib/control/PID/quad_pids.cpp
        airlib/control/navigation/WaypointController.cpp
        airlib/control/navigation/P2PNav.cpp
        )

set(HEADER_FILES
        include/airlib/robot_defs.h
        include/airlib/control/ControlViz.h
        include/airlib/control/ControllerInterface.h
        include/airlib/control/pid/PID.h

        include/airlib/localization/StateObserver.h
        include/airlib/localization/Filters/ComplementaryFilter.h
        include/airlib/localization/Sensors/ApriltagLandmarks.h
        include/airlib/localization/filters.h
        include/airlib/localization/sensors.h
        include/airlib/localization/Filters/FilterBase.h
        include/airlib/localization/Sensors/SensorBase.h
        include/airlib/localization/Sensors/DummyState.h

        include/airlib/utility/LoggerCSV.h

        include/airlib/control/pid/quad_pids.h
        include/airlib/control/controller.h

        include/traj/root_finder.hpp
        include/traj/traj_min_jerk.hpp
        include/traj/traj_min_snap.hpp
        include/traj/waypoint_trajectory_interface.h
        include/traj/rapidcsv.h

        include/airlib/control/ControllerBase.h
        include/traj/trajectory_planner.h

        include/airlib/control/navigation/WaypointController.h
        include/airlib/control/navigation/P2PNav.h


        )

add_library(airlibBebop2 ${SOURCE_FILES} ${HEADER_FILES}

)

target_link_libraries(airlibBebop2 Eigen3::Eigen ${catkin_LIBRARIES} -pthread)
## Declare a C++ executables
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(bebop2_controller_node src/bebop2_controller_node.cpp ${SOURCE_FILES} ${HEADER_FILES}
        )

add_executable(bebop2_state_estimator_lpf
        src/bebop2_state_estimator_lpf.cpp)

add_executable(bebop2_sim src/bebop2_sim_node.cpp ${SOURCE_FILES} ${HEADER_FILES})

## Specify libraries to link a library or executable target against
target_link_libraries(bebop2_controller_node airlibBebop2 -pthread)
target_link_libraries(bebop2_state_estimator_lpf airlibBebop2 -pthread)
target_link_libraries(bebop2_sim airlibBebop2 -pthread)

## ACTION SERVER

add_executable(waypoint_control_server src/waypoint_control_server.cpp ${SOURCE_FILES} ${HEADER_FILES})

target_link_libraries(
        waypoint_control_server
        ${catkin_LIBRARIES}
        ${Boost_LIBRARIES}
)


add_executable(p2p_navigation_server src/p2p_navigation_server.cpp ${SOURCE_FILES} ${HEADER_FILES})

target_link_libraries(
        p2p_navigation_server
        ${catkin_LIBRARIES}
        ${Boost_LIBRARIES}
)


#add_subdirectory(bebop2_controller_gui)
include_directories(include/catch2-2.7.0)
add_library(Catch INTERFACE)
add_executable( bebop2_controller_test
    test/test_apriltag_landmark.cpp ${SOURCE_FILES} ${HEADER_FILES}
    test/test.cpp
        test/test_quad_pids.cpp
        test/helper.h
        test/test_waypoint_trajectory.cpp
)
target_link_libraries(bebop2_controller_test Catch Eigen3::Eigen ${catkin_LIBRARIES} -pthread)

enable_testing()
add_test( NAME bebop2_controller_test COMMAND bebop2_controller_test )

