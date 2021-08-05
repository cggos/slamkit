# Mapping

-----

* create pointcloud map and octomap in files
  ```sh
  mkdir build
  cd build
  ./mapping_rgbd
  ```

* create 2D and 3D occupancy grid map with **octomap_server**
  ```sh
  # in build dir
  ROS_HOME=`pwd` roslaunch ../test_octomap.launch
  ```