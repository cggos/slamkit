# Robot Mapping

-----

[TOC]

# Metric Map


## Grid Map

* [ANYbotics/grid_map](https://github.com/ANYbotics/grid_map): Universal grid map library for mobile robotic mapping
* [占据栅格地图（Occupancy Grid Map）](https://zhuanlan.zhihu.com/p/21738718)

### 2D Occupancy Grid Map

* [costmap_2d (ROS Wiki)](http://wiki.ros.org/costmap_2d): provides an implementation of a 2D costmap that takes in sensor data from the world, builds a 2D or 3D occupancy grid of the data (depending on whether a voxel based implementation is used), and inflates costs in a 2D costmap based on the occupancy grid and a user specified inflation radius

### 3D Occupancy Grid Map

* [OctoMap](https://octomap.github.io/): An Efficient Probabilistic 3D Mapping Framework Based on Octrees, implements a 3D occupancy grid mapping approach, providing data structures and mapping algorithms in C++ particularly suited for robotics

* [ANYbotics/elevation_mapping](https://github.com/ANYbotics/elevation_mapping): Robot-centric elevation mapping for rough terrain navigation

* [ethz-asl/volumetric_mapping](https://github.com/ethz-asl/volumetric_mapping): 3D volumetric (occupancy) maps, providing a generic interface for disparity map and pointcloud insertion, and support for custom sensor error models

# Topological Map

# Semantic Map

# Map Viewers

* Octomap: `octovis octomap.bt`
* PCD: `pcl_viewer map.pcd`
