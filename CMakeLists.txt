cmake_minimum_required(VERSION 3.20)
project(TrajGen)

set(CMAKE_CXX_STANDARD 17)
find_package(Eigen3 REQUIRED)
add_subdirectory(pybind11)
pybind11_add_module(py_traj_gen SHARED main.cpp)
target_link_libraries(py_traj_gen PRIVATE
        Eigen3::Eigen
        pthread
)