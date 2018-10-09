cmake_minimum_required(VERSION 3.12)
project(ahrs_c_pcsim_gd)

set(CMAKE_CXX_STANDARD 14)

include_directories(./inc)

set(
        SOURCE_FILES
        ./src/training.cpp
)
add_library(training ${SOURCE_FILES})