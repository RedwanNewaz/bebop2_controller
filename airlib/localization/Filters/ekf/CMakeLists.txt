add_executable(ekf main.cpp ExtendedKalmanFilter.cpp ../../../../include/airlib/localization/Filters/ExtendedKalmanFilter.h ../include/filters_common/helper.h ../include/filters_common/matplotlibcpp.h)

target_link_libraries(ekf
        -lpython3.7m
        Eigen3::Eigen)