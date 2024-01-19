set(model_HEADER_FILES
    kinematic/ConstantVelocities.h
)

set(model_SRC_FILES
        kinematic/ConstantVelocities.cpp
)

add_library(airlib_model ${model_HEADER_FILES} ${model_SRC_FILES})
target_link_libraries(airlib_model
        Eigen3::Eigen
   )