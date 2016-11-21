cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(tester)

find_package(PCL 1.2 REQUIRED)
find_package( OpenCV REQUIRED )

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (tester tester.cpp ../common.cpp)
target_link_libraries (tester ${PCL_LIBRARIES} ${OpenCV_LIBS})
