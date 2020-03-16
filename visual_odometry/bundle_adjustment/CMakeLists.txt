# cmake_minimum_required(VERSION 2.8)

project(bundle_adjustment)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

set(CMAKE_CXX_STANDARD 11)

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

find_package(Cholmod REQUIRED)
include_directories(${CHOLMOD_INCLUDE_DIR})
# ${CHOLMOD_LIBRARIES}

add_subdirectory(common)

add_subdirectory(ba_gauss_newton)
add_subdirectory(ba_demo_ceres)

add_executable(bal_ceres bal_ceres.cpp)
target_link_libraries(bal_ceres bal_problem parse_cmd ${CERES_LIBRARIES})

add_executable(bal_g2o bal_g2o.cpp)
target_link_libraries(bal_g2o bal_problem parse_cmd ${G2O_LIBS} ${CHOLMOD_LIBRARIES})

add_executable(bal_draw_hessian bal_draw_hessian.cpp)
target_link_libraries(bal_draw_hessian bal_problem parse_cmd ${OpenCV_LIBS})
