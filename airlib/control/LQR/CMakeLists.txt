find_package(Eigen3 REQUIRED)
include_directories(${Eigen3_INCLUDE_DIRS})
add_executable(LQR main.cpp
        LQRController.cpp
        LQRController.h
       )
target_link_libraries(LQR Eigen3::Eigen)