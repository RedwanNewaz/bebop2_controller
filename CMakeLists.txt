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
)

## Declare a C++ executables
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide

add_subdirectory(perception)
add_subdirectory(controller)
add_subdirectory(navigation)

#add_subdirectory(bebop2_controller_gui)
add_subdirectory(test)

