project( mono_vo )

include_directories( include )

add_executable( mono_vo_01 mono_vo_01.cpp)
target_link_libraries( mono_vo_01 ${OpenCV_LIBS} )

add_executable( mono_vo_02 main.cpp src/visual_odometry.cpp src/pinhole_camera.cpp )
target_link_libraries( mono_vo_02 ${OpenCV_LIBS} )
