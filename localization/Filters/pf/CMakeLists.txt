

add_executable(pf_test main.cpp ../include/filters_common/helper.h particle_filter.cpp ../../../include/bebop2_controller/localization/Filters/particle_filter.h)

target_link_libraries(pf_test
        -lpython3.7m
        Eigen3::Eigen
        )