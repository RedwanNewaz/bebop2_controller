//
// Created by redwan on 9/30/23.
//

#ifndef BEBOP2_CONTROLLER_HELPER_H
#define BEBOP2_CONTROLLER_HELPER_H
#include <catch.hpp>
#include <iostream>

// Define a custom matcher for comparing floating-point vectors with a tolerance
template <typename T>
inline bool vectorsApprox(const std::vector<T>& v1, const std::vector<T>& v2, T epsilon) {
    if (v1.size() != v2.size()) {
        return false;
    }
    for (size_t i = 0; i < v1.size(); ++i) {
        if (Approx(v1[i]).epsilon(epsilon) != v2[i]) {
            return false;
        }
    }
    return true;
}

// Custom matcher function to compare two arrays of doubles with tolerance
inline bool areArraysEqualWithTolerance(const std::vector<double>& expected,
                                 const std::vector<double>& actual,
                                 double tolerance) {
    if (expected.size() != actual.size()) {
        return false;
    }

    for (size_t i = 0; i < expected.size(); ++i) {
        // Check if the absolute difference is less than the tolerance
        if (std::abs(expected[i] - actual[i]) > tolerance) {
            printf("[!] does not match (%lf == %lf) \n", expected[i], actual[i]);
            return false;
        }
    }

    return true;
}

#endif //BEBOP2_CONTROLLER_HELPER_H
