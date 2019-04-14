# Robot Calibration

* [mikeferguson/robot_calibration](https://github.com/mikeferguson/robot_calibration): Generic calibration for robots
* [pangfumin/odom_extrinsic_calibrate](https://github.com/pangfumin/odom_extrinsic_calibrate): Odom intrinsic and odom-sensors extrinsic calibration on SE2 using batch optimization

-----

[TOC]

# Hand-Eye Calibration

* [ethz-asl/hand_eye_calibration](https://github.com/ethz-asl/hand_eye_calibration):
Python tools to perform hand-eye calibration

* [jhu-lcsr/handeye_calib_camodocal](https://github.com/jhu-lcsr/handeye_calib_camodocal)(ROS + CamOdoCal Hand Eye Calibration): Easy to use and accurate hand eye calibration which has been working reliably for years (2016-present) with kinect, kinectv2, rgbd cameras, optical trackers, and several robots including the ur5 and kuka iiwa.


# IMU-Cam Calibration

## Kalibr

[Kalibr](https://github.com/ethz-asl/kalibr) is a toolbox that solves the following calibration problems:  

* Multiple camera calibration
* Camera-IMU calibration
* Rolling Shutter Camera calibration

My Blog: [Kalibr 之 Camera-IMU 标定 (总结)](https://blog.csdn.net/u011178262/article/details/83316968)

### TUM

* [Double Sphere Camera Model](https://vision.in.tum.de/research/vslam/double-sphere)
* [Visual-Inertial Dataset](https://vision.in.tum.de/data/datasets/visual-inertial-dataset)

## InerVis Toolbox for Matlab

* http://home.deec.uc.pt/~jlobo/InerVis_WebIndex/InerVis_Toolbox.html

**IMU CAM calibration**, Inertial Measurement Unit and Camera Calibration Toolbox

<div align=center>
  <img src="http://home.deec.uc.pt/~jlobo/InerVis_WebIndex/imu_cam_menu.gif">
</div>

## vicalib

* [vicalib (arpg)](https://github.com/arpg/vicalib)

# Lidar-Cam Calibration

* Camera and Range Sensor Calibration Toolbox: http://www.cvlibs.net/software/calibration/

* The [velo2cam_calibration](https://github.com/beltransen/velo2cam_calibration) software implements an Automatic Calibration algorithm for Lidar-Stereo camera setups

* [MegviiRobot/CamLaserCalibraTool](https://github.com/MegviiRobot/CamLaserCalibraTool): Extrinsic Calibration of a Camera and 2d Laser

* [ankitdhall/lidar_camera_calibration](https://github.com/ankitdhall/lidar_camera_calibration): The package is used to calibrate a Velodyne LiDAR with a camera (works for both monocular and stereo)


# Time Synchronization

* [chrony](https://chrony.tuxfamily.org/): is a versatile implementation of the Network Time Protocol (NTP). It can synchronise the system clock with NTP servers, reference clocks (e.g. GPS receiver), and manual input using wristwatch and keyboard. It can also operate as an NTPv4 (RFC 5905) server and peer to provide a time service to other computers in the network.  
* ROS [Clock](http://wiki.ros.org/Clock)
  ```bash
  rosparam set use_sim_time true  # (or set in launch file if you use one)
  rosbag play <your bag> --clock
  ```
* [TICSync](https://ori.ox.ac.uk/ticsync/) is an extremely efficient algorithm for learning the mapping between distributed clocks, which typically achieves better than millisecond accuracy within just a few seconds.
* [ethz-asl/cuckoo_time_translator](https://github.com/ethz-asl/cuckoo_time_translator)
* [leggedrobotics/hardware_time_sync](https://github.com/leggedrobotics/hardware_time_sync): Guidelines on how to hardware synchronize the time of multiple sensors, e.g., IMU, cameras, etc.
