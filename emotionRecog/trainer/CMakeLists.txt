cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(train_model)

find_package(PCL 1.2 REQUIRED)
find_package( OpenCV REQUIRED )

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (train_model train_model.cpp ../common.cpp)
target_link_libraries (train_model ${PCL_LIBRARIES} ${OpenCV_LIBS})
