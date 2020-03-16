add_executable(ba_gauss_newton ba_gauss_newton.cpp)
target_link_libraries(ba_gauss_newton ${Sophus_LIBRARIES})

add_executable(ba_gn_ceres ba_gn_ceres.cpp)
target_link_libraries(ba_gn_ceres ${Sophus_LIBRARIES} ${CERES_LIBRARIES})

add_executable(ba_gn_g2o ba_gn_g2o.cpp)
target_link_libraries(ba_gn_g2o ${Sophus_LIBRARIES} ${G2O_LIBS} ${CHOLMOD_LIBRARIES})