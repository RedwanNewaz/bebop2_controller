cmake_minimum_required(VERSION 3.5)
project(waypoints_sender_rviz)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++17 -DEIGEN_RUNTIME_NO_MALLOC")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -fno-asm")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -O0")

#############
## Library ##
#############
## Eigen ##
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
link_directories(${EIGEN3_LIBRARY_DIRS})

# Qt settings #
find_package(Qt5 ${rviz_QT_VERSION} REQUIRED Core Widgets)
macro(qt_wrap_ui)
  qt5_wrap_ui(${ARGN})
endmacro()
include_directories(${Qt5Core_INCLUDE_DIRS})
include_directories(${Qt5Widgets_INCLUDE_DIRS})
add_definitions(-DQT_NO_KEYWORDS)

# cmake settings #
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  class_loader
  geometry_msgs
  pluginlib
  roscpp
  rviz
  std_msgs
  bebop2_controller
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES waypoints_sender_rviz
 CATKIN_DEPENDS actionlib actionlib_msgs class_loader geometry_msgs pluginlib roscpp rviz std_msgs bebop2_controller
 DEPENDS system_lib EIGEN3
)


# add_dependencies(
#   ${PROJECT_NAME}
#   bebop2_controller_generate_messages_cpp
#   # ${bebop2_controller_EXPORTED_TARGETS}
# )
###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
 

)

# source codes #
set(HEADERS
  include/waypoints_sender_rviz/dial_panel.h
)

qt5_wrap_ui(UIC_FILES
  src/ui/dial_panel.ui
  )

set(SOURCE_FILES
  src/dial_panel.cpp
  )

## Declare a C++ library
add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES} ${HEADERS} ${UIC_FILES} )
set_target_properties(${PROJECT_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_include_directories(${PROJECT_NAME} PRIVATE "${OGRE_PREFIX_DIR}/include")
target_link_libraries(${PROJECT_NAME} -lpthread)

#############
## Install ##
#############
install(FILES plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY icons DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)