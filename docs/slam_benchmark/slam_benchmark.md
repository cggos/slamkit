# SLAM Benchmark

-----

## EVO

[evo](https://michaelgrupp.github.io/evo/): Python package for the evaluation of odometry and SLAM

> 1. get the ground truth file, e.g. EuRoC **data.csv**
> 2. convert it to TUM format **data.tum**: `evo_traj euroc data.csv --save_as_tum`
> 3. evaluate `evo_ape tum data.tum pose_out.tum --align --plot`
> 4. or directly `evo_ape euroc data.csv pose_out.tum --align --plot`


## rgbd-dataset tools

[Useful tools for the RGB-D benchmark (TUM)](https://vision.in.tum.de/data/datasets/rgbd-dataset/tools)

 
> 1. get the ground truth file, e.g. EuRoC **data.csv**
> 2. convert it to TUM format **data.tum**, we can use `evo`
> 3. evaluate: `./evaluate_ate.py data.tum pose_out.tum --plot result --verbose`
