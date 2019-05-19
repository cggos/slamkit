# SLAM Park

* [SLAM Blog (CSDN)](https://blog.csdn.net/u011178262/article/category/7456224)
* [Study Notes for SLAM (LaTeX on Overleaf)](https://www.overleaf.com/read/gtmwqbvfctkn)

-----

[TOC]

# EKF-SLAM

* MonoSLAM
  - [hanmekim/SceneLib2](https://github.com/hanmekim/SceneLib2): MonoSLAM open-source library

# vSLAM

* FOVIS: a visual odometry library that estimates the 3D motion of a camera using a source of depth information for each pixel.
  - Paper: *Visual Odometry and Mapping for Autonomous Flight Using an RGB-D Camera*
  - [libfovis](https://fovis.github.io/)
  - [fovis_ros](http://wiki.ros.org/fovis_ros): **mono_depth_odometer** and **stereo_odometer**

* SOFT

## Feature-based Method

* [cggos/ptam_cg](https://github.com/cggos/ptam_cg)

* [cggos/orbslam2_cg](https://github.com/cggos/orbslam2_cg)

* [cggos/viso2_cg](https://github.com/cggos/viso2_cg)

* [cggos/rgbdslam2_cg](https://github.com/cggos/rgbdslam2_cg)

* [rubengooj/pl-slam](https://github.com/rubengooj/pl-slam): an algorithm to compute stereo visual SLAM by using both point and line segment features

* [UcoSLAM](http://www.uco.es/investiga/grupos/ava/node/62) is a library for SLAM using keypoints that able to operate with **monocular cameras, stereo cameras, rgbd cameras**

## Semi-Direct Method

* [cggos/svo_cg](https://github.com/cggos/svo_cg): a **Semi-direct Monocular Visual Odometry** pipeline
* [HeYijia/svo_edgelet](https://github.com/HeYijia/svo_edgelet): A more robust SVO with edgelet feature

## Direct Method

* DSO
* [cggos/lsd_slam_cg](https://github.com/cggos/lsd_slam_cg)

# VI-SLAM

* [cggos/okvis_cg](https://github.com/cggos/okvis_cg)

* [cggos/vins_mono_cg](https://github.com/cggos/vins_mono_cg)

* [VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion): An optimization-based multi-sensor state estimator

* MSCKF
  - [daniilidis-group/msckf_mono](https://github.com/daniilidis-group/msckf_mono): Monocular MSCKF with ROS Support
  - [KumarRobotics/msckf_vio](https://github.com/KumarRobotics/msckf_vio): Robust Stereo Visual Inertial Odometry for Fast Autonomous Flight

* [cggos/rovio_cg](https://github.com/cggos/rovio_cg): Robust Visual Inertial Odometry

# Laser SLAM

* hector_slam
* gmapping

# SLAM Modules DIY

* [ROS CAMERA AND IMU SYNCHRONIZATION](http://grauonline.de/wordpress/?page_id=1951)

* [Visual inertial odometry on a budget!!](https://riccardogiubilato.github.io/visual/odometry/2017/12/12/Visual-Inertial-Odometry-On-A-Budget.html)
