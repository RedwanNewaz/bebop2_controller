
set(controller_HEADER_FILES
    ${PROJECT_SOURCE_DIR}/controller/interface/ControllerInterface.h
    ${PROJECT_SOURCE_DIR}/controller/interface/ControlViz.h
    ${PROJECT_SOURCE_DIR}/controller/LQR/LQRController.h
    ${PROJECT_SOURCE_DIR}/controller/LQR/quad_lqg.h
    ${PROJECT_SOURCE_DIR}/controller/PID/PID.h
    ${PROJECT_SOURCE_DIR}/controller/PID/quad_pids.h
)

set(controller_SRC_FILES
    ${PROJECT_SOURCE_DIR}/controller/interface/ControllerInterface.cpp
    ${PROJECT_SOURCE_DIR}/controller/interface/ControlViz.cpp
    ${PROJECT_SOURCE_DIR}/controller/LQR/LQRController.cpp
    ${PROJECT_SOURCE_DIR}/controller/LQR/quad_lqg.cpp
    ${PROJECT_SOURCE_DIR}/controller/PID/PID.cpp
    ${PROJECT_SOURCE_DIR}/controller/PID/quad_pids.cpp
)

add_executable(bebop2_controller_node interface/bebop2_controller_node.cpp
        ${controller_HEADER_FILES} ${controller_SRC_FILES}
)
target_link_libraries(bebop2_controller_node
        ${catkin_LIBRARIES}
        Eigen3::Eigen
        -pthread)