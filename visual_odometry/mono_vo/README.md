# Monocular Visual Odometry

## Algorithm

Uses Nister's Five Point Algorithm for Essential Matrix estimation, and FAST features, with a KLT tracker.

More details are available [here as a report](http://avisingh599.github.io/assets/ugp2-report.pdf), and [here as a blog post](http://avisingh599.github.io/vision/monocular-vo/).

Note that this project is not yet capable of doing reliable relative scale estimation,
so the scale informaion is extracted from the KITTI dataset ground truth files.

* [一个简单的视觉里程计实现](http://fengbing.net/2015/07/26/%E4%B8%80%E4%B8%AA%E7%AE%80%E5%8D%95%E7%9A%84%E8%A7%86%E8%A7%89%E9%87%8C%E7%A8%8B%E8%AE%A1%E5%AE%9E%E7%8E%B01/)

## Build

```bash
mkdir build
cd build
cmake ..
make
```

## Run

```bash
./mono_vo
```

## Before you run

In order to run this algorithm, you need to have either your own data, or else the sequences from [KITTI's Visual Odometry Dataset](http://www.cvlibs.net/datasets/kitti/eval_odometry.php).

In order to run this algorithm on your own data, you must modify the intrinsic calibration parameters in the code.
