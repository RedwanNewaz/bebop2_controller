add_executable(ekf main.cpp extended_kalman_filter.cpp extended_kalman_filter.h ../include/helper.h ../include/matplotlibcpp.h)

target_link_libraries(ekf
        -lpython3.7m
        Eigen3::Eigen)