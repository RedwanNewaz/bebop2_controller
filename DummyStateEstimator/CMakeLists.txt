set(DUMMY_SRC_FILES
        DummyState.cpp
        DummyState.h
        ${PROJECT_SOURCE_DIR}/controller/../Core/StateViz.h
        ${PROJECT_SOURCE_DIR}/controller/../Core/StateViz.cpp
        )



add_executable(bebop2_dummy_stateEstimator_node
        bebop2_dummy_stateEstimator_node.cpp ${DUMMY_SRC_FILES})

target_link_libraries(bebop2_dummy_stateEstimator_node
        ${catkin_LIBRARIES}
        )