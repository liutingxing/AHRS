cmake_minimum_required(VERSION 3.12)
project(ahrs_c_pcsim_gd)

set(CMAKE_CXX_STANDARD 14)

include_directories(
    ./inc
    ../../lib
)

set(
        SOURCE_FILES
        ./src/parser.cpp
        ./src/fusion.cpp
        ./src/training.cpp
)
add_library(parser ${SOURCE_FILES})