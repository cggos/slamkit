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

find_package( CSparse REQUIRED )
include_directories(${CSPARSE_INCLUDE_DIR})

find_package(Eigen3 REQUIRED)
if(Eigen3_FOUND)
    include_directories( ${Eigen3_INCLUDE_DIRS} )
endif()

find_package(Sophus)
if(Sophus_FOUND)
    include_directories( ${Sophus_INCLUDE_DIRS} )
    # link_libraries(Sophus)
endif()
list(APPEND Sophus_LIBRARIES Sophus)

find_package( OpenCV 3 REQUIRED COMPONENTS core imgproc highgui features2d calib3d video )
if( OpenCV_FOUND )
    include_directories( ${OpenCV_INCLUDE_DIRS} )
endif()

find_package(G2O REQUIRED)
if(G2O_FOUND)
    include_directories(${G2O_INCLUDE_DIRS})
    link_directories(/opt/g2o/lib)
endif()
set(G2O_LIBS g2o_core g2o_stuff g2o_types_sba g2o_solver_csparse g2o_csparse_extension)

# set(G2O_LIBS g2o_cli g2o_ext_freeglut_minimal g2o_simulator g2o_solver_slam2d_linear
#     g2o_types_icp g2o_types_slam2d g2o_types_sba g2o_types_slam3d g2o_core g2o_interface
#     g2o_solver_csparse g2o_solver_structure_only g2o_csparse_extension g2o_opengl_helper g2o_solver_dense
#     g2o_stuff g2o_types_sclam2d g2o_parser g2o_solver_pcg g2o_types_data g2o_types_sim3 cxsparse)

find_package( Pangolin )
include_directories( ${Pangolin_INCLUDE_DIRS} )

add_subdirectory(visual_tracking)
add_subdirectory(bundle_adjustment)
add_subdirectory(mono_vo)

add_library(pose_estimation SHARED pose_estimation.cpp)
target_link_libraries(pose_estimation ${OpenCV_LIBS} ${CSPARSE_LIBRARY} ${G2O_LIBS})

add_executable( pose_estimation_2d2d pose_estimation_2d2d.cpp )
target_link_libraries( pose_estimation_2d2d  pose_estimation)

add_executable( pose_estimation_3d2d pose_estimation_3d2d.cpp)
target_link_libraries( pose_estimation_3d2d pose_estimation )

add_executable( pose_estimation_3d3d_01 pose_estimation_3d3d_01.cpp)
target_link_libraries( pose_estimation_3d3d_01 pose_estimation )

add_executable(pose_estimation_3d3d_02 pose_estimation_3d3d_02.cpp)
target_link_libraries( pose_estimation_3d3d_02 ${Pangolin_LIBRARIES} ${Sophus_LIBRARIES} pose_estimation)

add_executable( direct_method direct_method.cpp )
target_link_libraries( direct_method ${OpenCV_LIBS} ${Pangolin_LIBRARIES} ${Sophus_LIBRARIES} )

add_executable(direct_sparse direct_sparse.cpp)
target_link_libraries(direct_sparse  ${OpenCV_LIBS} ${G2O_LIBS} pose_estimation)

add_executable(direct_semidense direct_semidense.cpp)
target_link_libraries(direct_semidense ${OpenCV_LIBS} ${G2O_LIBS} pose_estimation)

add_executable(e2rt e2rt.cpp epipolar_geometry.cpp)
target_link_libraries( e2rt ${Sophus_LIBRARIES} ${OpenCV_LIBS} )
