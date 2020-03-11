cmake_minimum_required( VERSION 2.8 )
project( vo1 )

set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

set(TUM_SRC ${CMAKE_CURRENT_SOURCE_DIR}/tum_data_rgbd.h)
if(NOT EXISTS ${TUM_SRC})
    file(DOWNLOAD
            https://raw.githubusercontent.com/cggos/slam_park_cg/master/slam_dataset/tum/tum_data_rgbd.h
            ${TUM_SRC}
            SHOW_PROGRESS)
endif()

list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

find_package( OpenCV 3.1 REQUIRED COMPONENTS core imgproc highgui features2d calib3d video )
include_directories( ${OpenCV_INCLUDE_DIRS} )

find_package( G2O REQUIRED )
include_directories( ${G2O_INCLUDE_DIRS} )
link_directories(/opt/g2o/lib)
set(G2O_USER_LIBS g2o_core g2o_stuff g2o_types_sba g2o_solver_csparse g2o_csparse_extension)

find_package( CSparse REQUIRED )
include_directories(${CSPARSE_INCLUDE_DIR})

find_package(Sophus)
if(Sophus_FOUND)
    include_directories( ${Sophus_INCLUDE_DIRS} )
endif()
list(APPEND Sophus_LIBRARIES Sophus)

find_package( Pangolin )
include_directories( ${Pangolin_INCLUDE_DIRS} )

find_package(Eigen3)
include_directories(${EIGEN3_INCLUDE_DIR})

add_library(pose_estimation SHARED pose_estimation.cpp)
target_link_libraries(pose_estimation
        ${OpenCV_LIBS}
        ${CSPARSE_LIBRARY}
        ${G2O_USER_LIBS})

add_executable( pose_estimation_2d2d pose_estimation_2d2d.cpp )
target_link_libraries( pose_estimation_2d2d  pose_estimation)

add_executable( pose_estimation_3d2d pose_estimation_3d2d.cpp)
target_link_libraries( pose_estimation_3d2d pose_estimation )

add_executable( pose_estimation_3d3d_01 pose_estimation_3d3d_01.cpp)
target_link_libraries( pose_estimation_3d3d_01 pose_estimation )

add_executable(pose_estimation_3d3d_02 pose_estimation_3d3d_02.cpp)
target_link_libraries( pose_estimation_3d3d_02 ${Pangolin_LIBRARIES} ${Sophus_LIBRARIES} pose_estimation)

add_executable(optical_flow_lk optical_flow_lk.cpp)
target_link_libraries(optical_flow_lk ${OpenCV_LIBS})

add_executable(optical_flow optical_flow.cpp optical_flow_test.cpp)
target_link_libraries(optical_flow ${OpenCV_LIBS})

add_executable( direct_method direct_method.cpp )
target_link_libraries( direct_method ${OpenCV_LIBS} ${Pangolin_LIBRARIES} ${Sophus_LIBRARIES} )

add_executable(direct_sparse direct_sparse.cpp)
target_link_libraries(direct_sparse  ${OpenCV_LIBS} ${G2O_USER_LIBS} pose_estimation)

add_executable(direct_semidense direct_semidense.cpp)
target_link_libraries(direct_semidense ${OpenCV_LIBS} ${G2O_USER_LIBS} pose_estimation)

add_executable(e2rt e2rt.cpp epipolar_geometry.cpp)
target_link_libraries( e2rt ${Sophus_LIBRARIES} ${OpenCV_LIBS} )
