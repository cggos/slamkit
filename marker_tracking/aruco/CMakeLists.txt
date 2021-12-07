cmake_minimum_required(VERSION 3.5)

project(aruco_demos)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-g)

find_package(OpenCV)
if(OpenCV_FOUND)
	include_directories(${OpenCV_INCLUDE_DIRS})
	link_libraries(${OpenCV_LIBS})
endif()

add_executable(aruco_detect_live aruco_detect_live.cpp)
add_executable(aruco_detect_img aruco_detect_img.cpp)
