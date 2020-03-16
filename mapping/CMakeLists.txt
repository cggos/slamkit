cmake_minimum_required( VERSION 2.8 )

set( CMAKE_BUILD_TYPE Release )
set( CMAKE_CXX_FLAGS "-std=c++11 -march=native -O3 -fopenmp" )

set(TUM_SRC ${CMAKE_CURRENT_SOURCE_DIR}/tum_data_rgbd.h)
if(NOT EXISTS ${TUM_SRC})
    file(DOWNLOAD
            https://raw.githubusercontent.com/cggos/slam_park_cg/master/slam_dataset/tum/tum_data_rgbd.h
            ${TUM_SRC}
            SHOW_PROGRESS)
endif()

# opencv
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

# eigen
include_directories( "/usr/include/eigen3/" )

# Sophus
find_package( Sophus REQUIRED )
include_directories( ${Sophus_INCLUDE_DIRS} )

# pcl
find_package( PCL REQUIRED COMPONENT common io filters )
include_directories( ${PCL_INCLUDE_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )

# octomap
find_package( octomap REQUIRED )
include_directories( ${OCTOMAP_INCLUDE_DIRS} )

add_executable( mapping_rgbd mapping_rgbd.cpp )
target_link_libraries( mapping_rgbd ${OpenCV_LIBS} ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES} )

add_executable(mapping_mono_dense mapping_mono_dense.cpp )
target_link_libraries( mapping_mono_dense ${OpenCV_LIBS} ${Sophus_LIBRARIES} )
