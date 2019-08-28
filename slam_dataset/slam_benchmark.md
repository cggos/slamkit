# SLAM Benchmark

-----

* [evo](https://michaelgrupp.github.io/evo/): Python package for the evaluation of odometry and SLAM
  - get the ground truth file, e.g. EuRoC **data.csv**
  - convert it to TUM format **data.tum**: `evo_traj euroc data.csv --save_as_tum`
  - evaluate: `evo_ape tum data.tum pose_out.tum --align --plot`

* [Useful tools for the RGB-D benchmark (TUM)](https://vision.in.tum.de/data/datasets/rgbd-dataset/tools)
  - get the ground truth file, e.g. EuRoC **data.csv**
  - convert it to TUM format **data.tum**, we can use `evo`
  - evaluate: `./evaluate_ate.py data.tum pose_out.tum --plot result --verbose`
