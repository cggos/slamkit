cmake_minimum_required( VERSION 2.8 )
project( eigen_demos )

set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

include_directories( "/usr/include/eigen3" )

find_package( Sophus REQUIRED )
include_directories( ${Sophus_INCLUDE_DIRS} )
set(Sophus_LIBRARIES Sophus)

find_package( Pangolin )
include_directories( ${Pangolin_INCLUDE_DIRS} )

add_executable( eigen_matrix eigen_matrix.cpp )

add_executable( eigen_geometry eigen_geometry.cpp )

add_executable( eigen_geometry_quickref eigen_geometry_quickref.cpp )

add_executable( eigen_geometry_visualize eigen_geometry_visualize.cpp )
target_link_libraries( eigen_geometry_visualize ${Pangolin_LIBRARIES} )

add_executable( eigen_sophus eigen_sophus.cpp )
target_link_libraries( eigen_sophus ${Sophus_LIBRARIES} )

add_executable( eigen_rotation eigen_rotation.cpp)
target_link_libraries( eigen_rotation ${Sophus_LIBRARIES} )

add_executable( eigen_quaternion_slerp eigen_quaternion_slerp.cpp )
