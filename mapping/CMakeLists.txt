cmake_minimum_required( VERSION 3.5 )

project(mapping)

set( CMAKE_BUILD_TYPE Debug )
set( CMAKE_CXX_FLAGS "-std=c++11 -march=native -O3 -fopenmp" )

set(TUM_SRC ${CMAKE_CURRENT_SOURCE_DIR}/tum_data_rgbd.h)
if(NOT EXISTS ${TUM_SRC})
    file(DOWNLOAD
            https://raw.githubusercontent.com/cggos/slam_park/master/slam_dataset/tum/tum_data_rgbd.h
            ${TUM_SRC}
            SHOW_PROGRESS)
endif()

find_package(Eigen3 REQUIRED)
if(Eigen3_FOUND)
    include_directories( ${Eigen3_INCLUDE_DIRS} )
endif()

find_package(Sophus)
if(Sophus_FOUND)
    include_directories( ${Sophus_INCLUDE_DIRS} )
endif()

find_package( OpenCV 3 REQUIRED COMPONENTS core imgproc highgui )
if( OpenCV_FOUND )
    include_directories( ${OpenCV_INCLUDE_DIRS} )
endif()

find_package( PCL REQUIRED COMPONENTS common io filters )
if( PCL_FOUND )
    include_directories( ${PCL_INCLUDE_DIRS} )
    add_definitions( ${PCL_DEFINITIONS} )
endif()

find_package( octomap REQUIRED )
if( octomap_FOUND )
    include_directories( ${OCTOMAP_INCLUDE_DIRS} )
endif()

add_executable( mapping_rgbd mapping_rgbd.cpp )
target_link_libraries( mapping_rgbd ${OpenCV_LIBS} ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES} )

add_executable(mapping_mono_dense mapping_mono_dense.cpp)
target_link_libraries(mapping_mono_dense ${OpenCV_LIBS})
