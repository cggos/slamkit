cmake_minimum_required(VERSION 3.5)

project(apriltag_demos)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-g)

find_package(OpenCV)
if(OpenCV_FOUND)
	include_directories(${OpenCV_INCLUDE_DIRS})
	link_libraries(${OpenCV_LIBS})
endif()
Find_Package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)
list(APPEND Sophus_LIBRARIES Sophus)
find_package(Pangolin)
find_package(apriltag)
include_directories(${EIGEN3_INCLUDE_DIR} ${Sophus_INCLUDE_DIRS} ${Pangolin_INCLUDE_DIRS} ${apriltag_INCLUDE_DIRS})

add_executable(apriltag_opencv_demo apriltag_opencv_demo.cc)
target_link_libraries(apriltag_opencv_demo ${OpenCV_LIBRARIES} ${Sophus_LIBRARIES} ${Pangolin_LIBRARIES} apriltag)    
set_target_properties(apriltag_opencv_demo PROPERTIES CXX_STANDARD 11)