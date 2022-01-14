cmake_minimum_required(VERSION 3.01)
project(M2DP)
set(CMAKE_CXX_STANDARD 14 )
set( CMAKE_CXX_FLAGS -O3 )

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
include_directories( "/usr/include/eigen3" )
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable(m2dp m2dp.cpp)
target_link_libraries(m2dp ${PCL_LIBRARIES})
