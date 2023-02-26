
add_executable(test_odom odom_to_csv.cpp ../airlib/localization/Sensors/ApriltagLandmarks.cpp)
target_link_libraries(test_odom ${catkin_LIBRARIES})