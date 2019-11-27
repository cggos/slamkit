# SLAM 运行经验总结

-----

# 1. ORB-SLAM

# 2. HKUST VINS

* 启动运行即飞，原因主要有两个：
  - 初始化操作有误
  - cam和imu的外参不对

* VINS-Mono 和 VINS-Fusion 中 cam和imu的外参 为 **from camera frame to imu frame**，不要写反

## VINS-Mono

* 动态初始化，操作手法可以参考如下 **8字法** （下图为Suunto腕表校准指南针的手法）  

  <div align=center>
    <img src="images/suunto_watch_compass_calib.png"/>
  </div>

## VINS-Fusion

* 静态初始化

* VINS-Fusion 即使开启在线外参估计选项 `estimate_extrinsic: 1`，对应的cam和imu的外参 `body_T_cam0` 和 `body_T_cam1` 最好也要给一个比较好的初始参数，否则（比如单位阵）容易飞


# 3. OKVIS

# 4. MSCKF
