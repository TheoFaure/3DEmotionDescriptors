cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(data_grabber)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (data_grabber data_grabber.cpp ../common.cpp)
target_link_libraries (data_grabber ${PCL_LIBRARIES})
