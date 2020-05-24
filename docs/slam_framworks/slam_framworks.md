## SLAM Frameworks Overview

-----

[TOC]

### Filter-SLAM

#### EKF-SLAM

* [The Extended Kalman Filter - SLAM](https://www.doc.ic.ac.uk/~ns5517/topicsWebsite/EKF-SLAM.html)

* [EKF Slam Example](https://www.mathworks.com/matlabcentral/fileexchange/39992-ekf-slam-example)

* [EKF-SLAM-on-Manifold](https://github.com/RomaTeng/EKF-SLAM-on-Manifold)

* [damarquezg/SLAMTB](https://github.com/damarquezg/SLAMTB): EKF-SLAM TOOLBOX FOR MATLAB

##### MonoSLAM

* [cggos/SceneLib2](https://github.com/cggos/SceneLib2)
  - MonoSLAM open-source library
  - an open-source C++ library for SLAM originally designed and implemented by **Andrew Davison** and colleagues at the University of Oxford

* [MATLAB Implementation of MonoSLAM](http://perso.ensta-paristech.fr/~filliat/Courses/2011_projets_C10-2/BRUNEAU_DUBRAY_MURGUET/monoSLAM_bruneau_dubray_murguet_en.html)

* [EKF based Monocular SLAM](http://vision.ia.ac.cn/Students/gzp/monocularslam.html)

* [Mono-Slam Implementation in ROS](https://github.com/rrg-polito/mono-slam)

##### EKFmonocularSLAM

* [EKFmonocularSLAM](https://openslam-org.github.io/ekfmonoslam.html)

* [1-Point RANSAC Inverse Depth EKF Monocular SLAM](http://webdiis.unizar.es/~jcivera/code/1p-ransac-ekf-monoslam.html)



### vSLAM

* [Monocular SLAM](https://www.doc.ic.ac.uk/~ab9515/index.html)
* [mono_vo](https://github.com/cggos/slam_cg/tree/master/mono_vo)

* FOVIS: a visual odometry library that estimates the 3D motion of a camera using a source of depth information for each pixel.
  - Paper: *Visual Odometry and Mapping for Autonomous Flight Using an RGB-D Camera*
  - [libfovis](https://fovis.github.io/)
  - [fovis_ros](http://wiki.ros.org/fovis_ros): **mono_depth_odometer** and **stereo_odometer**

* SOFT


#### Feature-based Method

* [cggos/ptam_cg](https://github.com/cggos/ptam_cg)

* [cggos/orbslam2_cg](https://github.com/cggos/orbslam2_cg)

* [cggos/viso2_cg](https://github.com/cggos/viso2_cg)

* [cggos/rgbdslam2_cg](https://github.com/cggos/rgbdslam2_cg)

* [rubengooj/pl-slam](https://github.com/rubengooj/pl-slam): an algorithm to compute stereo visual SLAM by using both point and line segment features

* [UcoSLAM](http://www.uco.es/investiga/grupos/ava/node/62) is a library for SLAM using keypoints that able to operate with **monocular cameras, stereo cameras, rgbd cameras**

#### Semi-Direct Method

* [cggos/svo_cg](https://github.com/cggos/svo_cg): a **Semi-direct Monocular Visual Odometry** pipeline
* [HeYijia/svo_edgelet](https://github.com/HeYijia/svo_edgelet): A more robust SVO with edgelet feature

#### Direct Method

* DSO
* [cggos/lsd_slam_cg](https://github.com/cggos/lsd_slam_cg)


### VI-SLAM/VIO

* [cggos/okvis_cg](https://github.com/cggos/okvis_cg)

* [cggos/vins_mono_cg](https://github.com/cggos/vins_mono_cg)

* [cggos/vins_fusion_cg](https://github.com/cggos/vins_fusion_cg)

* [cggos/msckf_cg](https://github.com/cggos/msckf_cg)

* [cggos/rovio_cg](https://github.com/cggos/rovio_cg): Robust Visual Inertial Odometry


### Laser SLAM

* [rplidar_ros](https://github.com/robopeak/rplidar_ros.git) (branch `slam`)
* [hector_slam](https://github.com/tu-darmstadt-ros-pkg/hector_slam.git)
* [openslam_gmapping](https://github.com/OpenSLAM-org/openslam_gmapping.git)
* [slam_gmapping](https://github.com/ros-perception/slam_gmapping.git)

Example Run:

* 通过 **hector_slam** 建图，运行 `roslaunch rplidar_ros view_slam.launch`，效果如下
  ![](images/hector_slam.jpg)
