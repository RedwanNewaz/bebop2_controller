#set(STATE_SRC_FILES
#        src/StateEstimator.h src/SensorData.cpp src/SensorData.h src/StateEstimator.cpp ../Core/StateViz.h ../Core/StateViz.cpp
#        )
## Declare a C++ library
#add_library(${PROJECT_NAME}
#        src/bebop2_stateEstimator_nodelet.cpp
#        ${STATE_SRC_FILES}
#        )


add_executable(bebop2_stateEstimator_node src/bebop2_stateEstimator_node.cpp ${STATE_SRC_FILES})

## Specify libraries to link a library or executable target against
target_link_libraries(bebop2_stateEstimator_node
        ${catkin_LIBRARIES}
        )
