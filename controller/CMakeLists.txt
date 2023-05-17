#set(CONTROLLER_SRC_FILES
#        src/pid.cpp src/pid.h src/JoystickInterface.cpp
#        src/JoystickInterface.h ../Core/StateViz.h ../Core/StateViz.cpp src/DroneController.cpp src/DroneController.h
#        )
#
### Declare a C++ library
#add_library(${PROJECT_NAME}
#        src/bebop2_controller_nodelet.cpp
#        ${CONTROLLER_SRC_FILES}
#        )


add_executable(bebop2_controller_node
        src/bebop2_controller_node.cpp ${CONTROLLER_SRC_FILES})

target_link_libraries(bebop2_controller_node
        ${catkin_LIBRARIES}
        )