
set(perception_HEADER_FILES
        ${PROJECT_SOURCE_DIR}/perception/sensor/landmark/ApriltagLandmarks.h
        ${PROJECT_SOURCE_DIR}/perception/sensor/dummy/DummyState.h
        ${PROJECT_SOURCE_DIR}/perception/filter/LPF/ComplementaryFilter.h
        ${PROJECT_SOURCE_DIR}/perception/interface/StateObserver.h

)

set(perception_SRC_FILES
        ${PROJECT_SOURCE_DIR}/perception/sensor/landmark/ApriltagLandmarks.cpp
        ${PROJECT_SOURCE_DIR}/perception/sensor/dummy/DummyState.cpp
        ${PROJECT_SOURCE_DIR}/perception/interface/StateObserver.cpp
)

add_executable(bebop2_state_estimator_lpf ${perception_HEADER_FILES} ${perception_SRC_FILES}
        interface/bebop2_state_estimator_lpf_node.cpp)

target_link_libraries(bebop2_state_estimator_lpf
        ${catkin_LIBRARIES}
        Eigen3::Eigen
        -pthread)

add_executable(bebop2_state_estimator_vicon ${perception_HEADER_FILES} ${perception_SRC_FILES}
        interface/bebop2_state_estimator_vicon_node.cpp)

target_link_libraries(bebop2_state_estimator_vicon
        ${catkin_LIBRARIES}
        Eigen3::Eigen
        -pthread)



add_executable(bebop2_sim ${perception_HEADER_FILES} ${perception_SRC_FILES}
        interface/bebop2_sim_state_node.cpp)

target_link_libraries(bebop2_sim
        ${catkin_LIBRARIES}
        Eigen3::Eigen
        -pthread)

add_library(airlib_percept ${perception_HEADER_FILES} ${perception_SRC_FILES})
target_link_libraries(airlib_percept
        ${catkin_LIBRARIES}
        Eigen3::Eigen
        -pthread)


