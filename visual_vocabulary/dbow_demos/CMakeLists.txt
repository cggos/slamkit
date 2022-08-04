cmake_minimum_required( VERSION 2.8 )
project( visual_vocabulary )

set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

option(WITH_DBOW2 "use DBoW2" ON)
option(WITH_DBOW3 "use DBoW3" OFF)

find_package(OpenCV 3 REQUIRED)
list(APPEND LIBS ${OpenCV_LIBS})
include_directories(${OpenCV_INCLUDE_DIRS})

if(WITH_DBOW2)
  add_definitions(-DWITH_DBOW2)
  find_package(DBoW2 REQUIRED)
  list(APPEND LIBS ${DBoW2_LIBRARIES})
endif()
if(WITH_DBOW3)
  add_definitions(-DWITH_DBOW3)
  set(DBoW3_INCLUDE_DIRS "/usr/local/include")
  set(DBoW3_LIBS "/usr/local/lib/libDBoW3.so")
  list(APPEND LIBS ${DBoW3_LIBS})
endif()

add_executable(create_voc create_voc.cpp)
target_link_libraries(create_voc ${LIBS})

add_executable(loop_closure loop_closure.cpp)
target_link_libraries(loop_closure ${LIBS})
