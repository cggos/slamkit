cmake_minimum_required(VERSION 3.5)

project(aruco_detection)

set(CMAKE_CXX_STANDARD 11)

find_package(OpenCV)
if(OpenCV_FOUND)
	include_directories(${OpenCV_INCLUDE_DIRS})
	link_libraries(${OpenCV_LIBS})
endif()

set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-g)

add_executable(aruco_detection aruco_detection.cpp)