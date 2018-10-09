cmake_minimum_required(VERSION 3.12)
project(ahrs_c_pcsim_gd)

set(CMAKE_CXX_STANDARD 14)

include_directories(./inc)
include_directories(../training/inc)
include_directories(../../lib/eigen)

set(
        SOURCE_FILES
        ./src/fusion.cpp
)
add_library(fusionon ${SOURCE_FILES})