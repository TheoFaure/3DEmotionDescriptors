set (CMAKE_CXX_STANDARD 11)
cmake_minimum_required(VERSION 3.1.0 FATAL_ERROR)

project(classifier)

find_package( OpenCV REQUIRED )

add_executable (classifier classifier.cpp)
target_link_libraries (classifier ${OpenCV_LIBS})

