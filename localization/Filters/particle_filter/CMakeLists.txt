cmake_minimum_required(VERSION 3.15)
project(ParticleFilter)

set(CMAKE_CXX_STANDARD 17)

include_directories(
        include
        ${CMAKE_BINARY_DIR}/matplotlib-prefix/src/matplotlib
        /usr/local/Cellar/python/3.7.1/Frameworks/Python.framework/Versions/3.7/include/python3.7m/
        /usr/local/lib/python3.7/site-packages/numpy/core/include/
)
find_package(Python3 REQUIRED)
find_package(Eigen3 REQUIRED)
link_directories(
        /Users/redwannewaz/anaconda3/lib/
)

add_executable(ParticleFilter main.cpp helper.h particle_filter.cpp particle_filter.h)

target_link_libraries(${PROJECT_NAME}
        -lpython3.7m
        Eigen3::Eigen
        )
