cmake_minimum_required( VERSION 2.8 )
project( visual_vocabulary )

set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

find_package( OpenCV 3.1 REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

# dbow3 is a simple lib so I assume you installed it in default directory
set( DBoW3_INCLUDE_DIRS "/usr/local/include" )
set( DBoW3_LIBS "/usr/local/lib/libDBoW3.so" )

add_executable( create_vocab_01 create_vocab_01.cpp )
target_link_libraries( create_vocab_01 ${OpenCV_LIBS} ${DBoW3_LIBS} )

add_executable( create_vocab_02 create_vocab_02.cpp )
target_link_libraries( create_vocab_02 ${OpenCV_LIBS} ${DBoW3_LIBS} )

add_executable( loop_closure loop_closure.cpp )
target_link_libraries( loop_closure ${OpenCV_LIBS} ${DBoW3_LIBS} )
