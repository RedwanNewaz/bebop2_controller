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
find_package(Python3 REQUIRED)


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
#  LIBRARIES bebop2_controller
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
  ${catkin_INCLUDE_DIRS}
   /usr/include/python3.8/
)

set(HEADER_FILES
        include/bebop2_controller/robot_defs.h
        include/bebop2_controller/control/ControlViz.h
        include/bebop2_controller/control/ControllerBase.h
        include/bebop2_controller/control/PID.h
        include/bebop2_controller/control/QuadControllerPID.h

        include/bebop2_controller/localization/StateObserver.h
        include/bebop2_controller/localization/Filters/ComplementaryFilter.h
        include/bebop2_controller/localization/Sensors/ApriltagLandmarks.h
        include/bebop2_controller/localization/filters.h
        include/bebop2_controller/localization/sensors.h
        include/bebop2_controller/localization/Filters/FilterBase.h
        include/bebop2_controller/localization/Sensors/SensorBase.h
        include/bebop2_controller/localization/Sensors/DummyState.h
        )


set(SOURCE_FILES
        src/bebop2_controller_node.cpp

        localization/Filters/ekf/ExtendedKalmanFilter.cpp
        localization/StateObserver.cpp
        localization/Sensors/ApriltagLandmarks.cpp
        localization/Sensors/DummyState.cpp
        control/ControlViz.cpp
        control/PID.cpp
        control/ControllerBase.cpp
        control/QuadControllerPID.cpp

        )
## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
 add_executable(${PROJECT_NAME}_node ${SOURCE_FILES} ${HEADER_FILES} )

## Specify libraries to link a library or executable target against
 target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
   Eigen3::Eigen
 )

