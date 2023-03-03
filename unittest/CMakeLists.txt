
add_executable(test_odom odom_to_csv.cpp
        ../airlib/localization/Sensors/ApriltagLandmarksExtended.cpp
        ../airlib/localization/Sensors/ApriltagLandmarks.cpp)
target_link_libraries(test_odom ${catkin_LIBRARIES})

add_executable(test_ekf test_ekf.cpp
        ../airlib/localization/Sensors/ApriltagLandmarks.cpp
        ../airlib/localization/Filters/ekf/ExtendedKalmanFilter.cpp
        )
target_link_libraries(test_ekf ${catkin_LIBRARIES})

add_executable(test_complimentary_filter test_complimentary_filter.cpp
        ../airlib/localization/Sensors/ApriltagLandmarks.cpp
        ../include/airlib/localization/Filters/ComplementaryFilter.h
        )
target_link_libraries(test_complimentary_filter ${catkin_LIBRARIES})


add_executable(test_robot_localization test_robot_localization.cpp
        ../airlib/localization/Sensors/ApriltagLandmarksExtended.cpp
        ../airlib/localization/Filters/RobotLocalization.cpp
        )
target_link_libraries(test_robot_localization ${catkin_LIBRARIES})


