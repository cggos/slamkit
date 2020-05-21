## Visual Vocabulary

* [Simple bag-of-words loop closure for visual SLAM](https://nicolovaligi.com/bag-of-words-loop-closure-visual-slam.html)

-----

[TOC]

### DBoW

* [dorian3d/DBoW](https://github.com/dorian3d/DBow): an open source C++ library for indexing and converting images into a bag-of-word representation (Note: out of date)
* [dorian3d/DBoW2](https://github.com/dorian3d/DBoW2): an improved version of the DBow library
* [rmsalinas/DBow3](https://github.com/rmsalinas/DBow3): an improved version of the DBow2 library
  ```bash
  git clone https://github.com/rmsalinas/DBow3.git
  cd DBow3
  mkdir build & cd build
  cmake .. & make
  ```

### DLoopDetector

* [dorian3d/DLoopDetector](https://github.com/dorian3d/DLoopDetector) is an open source C++ library to detect loops in a sequence of images collected by a mobile robot

### FBOW

* [rmsalinas/fbow](https://github.com/rmsalinas/fbow): FBOW (Fast Bag of Words) is an extremmely optimized version of the DBow2/DBow3 libraries.

### FAB-MAP

* [FAB-MAP](http://www.robots.ox.ac.uk/~mjc/Software.htm) is a system for **appearance-based** navigation or place recognition

* [arrenglover/openfabmap](https://github.com/arrenglover/openfabmap): Open-source C++ code for the FAB-MAP visual place recognition algorithm

* [OpenFABMAP (OpenCV)](https://docs.opencv.org/2.4/modules/contrib/doc/openfabmap.html) is an open and modifiable code-source which implements the **Fast Appearance-based Mapping algorithm (FAB-MAP)** developed by Mark Cummins and Paul Newman


### Visual Vocabulary Example

* ORBvoc.txt.tar.gz
  ```sh
  # location: https://github.com/raulmur/ORB_SLAM2/blob/master/Vocabulary/
  wget https://raw.githubusercontent.com/raulmur/ORB_SLAM2/master/Vocabulary/ORBvoc.txt.tar.gz
  ```
