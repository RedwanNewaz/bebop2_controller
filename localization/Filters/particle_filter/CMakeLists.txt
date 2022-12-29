

add_executable(pf_test main.cpp ../include/helper.h particle_filter.cpp particle_filter.h)

target_link_libraries(pf_test
        -lpython3.7m
        Eigen3::Eigen
        )