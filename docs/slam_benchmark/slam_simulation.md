# SLAM Simulation

-----

## VIO Simulator

* 思路 1：指定轨迹方程，求一阶导得到速度和角速度，求二阶导得到加速度
  - [HeYijia/vio_data_simulation](https://github.com/HeYijia/vio_data_simulation): Generate imu data and feature in camera frame

* 思路 2：已有pose轨迹，不知道方程，利用 B-Spline 产生IMU数据
  * from OpenVINS (On manifold SE3 B-Spline)
    - https://github.com/rpng/open_vins/tree/master/ov_msckf/src/sim
    - https://github.com/Edwinem/VI-Simulator
  