cmake_minimum_required(VERSION 3.0.2)
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

)

find_package(Eigen3 REQUIRED)

#add_subdirectory(control/LQR)
#add_subdirectory(control/PID)
###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
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
   /usr/include/python3.8/
)

set(SOURCE_FILES
        airlib/localization/Filters/ekf/ExtendedKalmanFilter.cpp
        airlib/localization/StateObserver.cpp
        airlib/localization/Sensors/ApriltagLandmarks.cpp
        airlib/localization/Sensors/DummyState.cpp
        airlib/control/ControlViz.cpp
        airlib/control/PID.cpp
        airlib/control/ControllerBase.cpp
        airlib/control/QuadControllerPID.cpp
        )

set(HEADER_FILES
        include/airlib/robot_defs.h
        include/airlib/control/ControlViz.h
        include/airlib/control/ControllerBase.h
        include/airlib/control/PID.h
        include/airlib/control/QuadControllerPID.h

        include/airlib/localization/StateObserver.h
        include/airlib/localization/Filters/ComplementaryFilter.h
        include/airlib/localization/Sensors/ApriltagLandmarks.h
        include/airlib/localization/filters.h
        include/airlib/localization/sensors.h
        include/airlib/localization/Filters/FilterBase.h
        include/airlib/localization/Sensors/SensorBase.h
        include/airlib/localization/Sensors/DummyState.h
        )

add_library(airlibBebop2 ${SOURCE_FILES} ${HEADER_FILES} )
target_link_libraries(airlibBebop2 Eigen3::Eigen ${catkin_LIBRARIES})
## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(apriltag_complementary_pid_node src/apriltag_complementary_pid.cpp  )
add_executable(apriltag_ekf_pid_node src/apriltag_ekf_pid.cpp  )
add_executable(dummy_ekf_pid_node src/dummy_ekf_pid.cpp )

## Specify libraries to link a library or executable target against
target_link_libraries(apriltag_complementary_pid_node airlibBebop2 )
target_link_libraries(apriltag_ekf_pid_node airlibBebop2)
target_link_libraries(dummy_ekf_pid_node airlibBebop2)

