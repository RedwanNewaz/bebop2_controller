find_package(Eigen3 REQUIRED)
include_directories(${Eigen3_INCLUDE_DIRS})
add_executable(LQR main.cpp LQR.cpp LQR.h)
target_link_libraries(LQR Eigen3::Eigen)