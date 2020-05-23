## Visual Odometry

-----

[TOC]

### Visual Tracking

#### Optical Flow

* LK

### Pose Estimation

#### 2D-2D: Epipolar Geometry

* Fundamental Matrix
* Essential Matrix
* Homography Matrix

#### 3D-3D: ICP

* SVD
* 非线性优化

#### 3D-2D: PnP

The **Perspective-n-Point (PnP)** pose problem is the problem of estimating the relative pose – 3D position and orientation – between a calibrated perspective camera and a 3D object (or between the camera and the entire 3D scene) from a set of n visible 3D points with known (X,Y,Z) object (or scene) coordinates and their 2D projections with known (u,v) pixel coordinates.

The PnP problem is relevant in 3D object tracking and camera localization / tracking and is often used for example in Structure-from-Motion (SfM), Visual Odometry (VO), Simultaneous Localization and Mapping (SLAM) and image-based localization pipelines.

* [Perspective-n-Point (Wikipedia)](https://en.wikipedia.org/wiki/Perspective-n-Point)

* Code
  - [midjji/pnp](https://github.com/midjji/pnp): A RANSAC and BA based pnp wrapper for the Lambdatwist p3p solver
  - [ydsf16/PnP_Solver](https://github.com/ydsf16/PnP_Solver): Personal implementations of solvers for PnP problem, including DLT and EPnP

##### Solver

* DLT
* P3P
* EPnP
* UPnP
* Motion Only BA（非线性优化）
* RANSAC


#### Direct Method VO

