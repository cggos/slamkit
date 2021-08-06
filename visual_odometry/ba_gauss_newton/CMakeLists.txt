add_executable(ba_gn ba_gauss_newton.cpp)
target_link_libraries(ba_gn)

add_executable(ba_gn_ceres ba_gn_ceres.cpp)
target_link_libraries(ba_gn_ceres ${CERES_LIBRARIES})

add_executable(ba_gn_ceres_q ba_gn_ceres_q.cpp)
target_link_libraries(ba_gn_ceres_q ${CERES_LIBRARIES})

add_executable(ba_gn_g2o ba_gn_g2o.cpp)
target_link_libraries(ba_gn_g2o ${G2O_LIBS} ${CHOLMOD_LIBRARIES})