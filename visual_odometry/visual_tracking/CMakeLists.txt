project(visual_tracking)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

set(CMAKE_CXX_STANDARD 11)

add_executable(optical_flow_lk optical_flow_lk.cpp)
target_link_libraries(optical_flow_lk ${OpenCV_LIBS})

add_executable(optical_flow optical_flow.cpp optical_flow_test.cpp)
target_link_libraries(optical_flow ${OpenCV_LIBS})