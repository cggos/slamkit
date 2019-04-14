# TUM Data and Tools

-----

[TOC]

## RGB-D SLAM Dataset and Benchmark

* the [RGB-D datasets](https://vision.in.tum.de/data/datasets/rgbd-dataset) from the Kinect

### associate_py

* 匹配rgb和depth图像时间戳
  ```sh
  python associate.py rgb.txt depth.txt > associate.txt
  ```

* 匹配associate.txt和groundtruth.txt中的时间信息
  ```sh
  python associate.py associate.txt groundtruth.txt > associate_with_groundtruth.txt
  ```

### File Formats

https://vision.in.tum.de/data/datasets/rgbd-dataset/file_formats

* Intrinsic Camera Calibration
* depth scale

```c++
fx = 525.0  # focal length x
fy = 525.0  # focal length y
cx = 319.5  # optical center x
cy = 239.5  # optical center y

factor = 5000 # for the 16-bit PNG files
# OR: factor = 1 # for the 32-bit float images in the ROS bag files

for v in range(depth_image.height):
  for u in range(depth_image.width):
    Z = depth_image[v,u] / factor;
    X = (u - cx) * Z / fx;
    Y = (v - cy) * Z / fy;
```

### Cpp Code

* tum_data_rgbd.h
