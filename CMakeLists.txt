cmake_minimum_required(VERSION 3.23)
project(LQR)

set(CMAKE_CXX_STANDARD 17)
find_package(Eigen3 REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})
add_executable(LQR main.cpp LQR.cpp LQR.h)
target_link_libraries(LQR Eigen3::Eigen)
