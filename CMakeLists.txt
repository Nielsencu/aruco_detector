# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2019 Intel Corporation. All Rights Reserved.
cmake_minimum_required(VERSION 3.1.0)

project(aruco_detector)

# Find librealsense2 installed package
find_package(realsense2 REQUIRED)
find_package(OpenCV REQUIRED)

# Enable C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

# Add the application sources to the target
add_executable(${PROJECT_NAME} aruco_detector.cpp)

include_directories( ${OpenCV_INCLUDE_DIRS} )

# Link librealsense2 to the target
target_link_libraries(${PROJECT_NAME} ${realsense2_LIBRARY})

target_link_libraries( ${PROJECT_NAME} ${OpenCV_LIBS} )