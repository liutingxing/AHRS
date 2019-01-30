cmake_minimum_required(VERSION 3.12)
project(spline_fitting)

set(CMAKE_CXX_STANDARD 14)

include_directories(
        ./lib
)

set(
        SOURCE_FILES
        ./src/spline_fitting.cpp
)

add_executable(spline_fitting ${SOURCE_FILES})