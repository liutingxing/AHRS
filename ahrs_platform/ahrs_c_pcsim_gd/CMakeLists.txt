cmake_minimum_required(VERSION 3.12)
project(ahrs_c_pcsim_gd)

set(CMAKE_CXX_STANDARD 14)

add_subdirectory(./module/parser)
include_directories(./module/parser/inc)
add_subdirectory(./module/training)
include_directories(./module/training/inc)
add_subdirectory(./module/fusion)
include_directories(./module/fusion/inc)


set(
        SOURCE_FILES
        ./test.cpp
)
add_executable(ahrs_c_pcsim_gd ${SOURCE_FILES})
target_link_libraries(ahrs_c_pcsim_gd parser)