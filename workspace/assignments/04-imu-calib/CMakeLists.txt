project(imu_calibration)

cmake_minimum_required(VERSION 3.15)
set(CMAKE_CXX_STANDARD 17)

find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)

#include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(test apps/test.cpp)
target_include_directories(test PRIVATE
    ${EIGEN3_INCLUDE_DIR}
    include)