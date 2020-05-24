.. SlamPark documentation master file, created by
   sphinx-quickstart on Tue May 19 23:25:56 2020.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Welcome to SlamPark's documentation!
====================================

* `Simultaneous localization and mapping (wikipedia)] <https://en.wikipedia.org/wiki/Simultaneous_localization_and_mapping/>`_

SLAM Study Notes:

* `SLAM Blog (CSDN) <https://blog.csdn.net/u011178262/article/category/7456224/>`_
* `SLAM Notes (github.io) <https://cggos.github.io/categories.html#SLAM>`_
* `SLAM Notes (LaTeX on Overleaf) <https://www.overleaf.com/read/drmrxvnphrck/>`_


SLAM Theoretical Basis
-------------------------

* Mathematics
  
  * https://github.com/cggos/maths_cg

* Computer Vision

  * https://github.com/cggos/cgocv
  * https://cggos.github.io/categories.html#ComputerVision

* Kinematics and Dynamics

* State Estimation
  
  * https://github.com/cggos/state_estimation_cg

* Sensors

  * Sensor Types
  
  * Sensor Calibration
    
    * https://cggos.github.io/robotics/robot-calibration.html



Localization & Mapping
-------------------------

* SLAM Frameworks

* SLAM Odometry

  * Visual Odometry/Visual Inertial Odometry
  
  * GNSS & INS
    
    * https://github.com/cggos/gnss_ins_cg

  * Wheel Odometry

* SLAM Loop Closure

  * Visual Vocabulary

* SLAM Mapping



Multiple Sensor Fusion
-------------------------

* https://github.com/cggos/sensor_fusion_cg



SLAM Benchmark
-------------------------

* SLAM Benchmark
* SLAM Dataset



SLAM Applications
-------------------------

* SLAM Modules
* AR (6 DoF)
* Drone (:math:`\approx` 4 DoF)
* Ground Mobile Robot (3 DoF)



SLAM QA
-------------------------

* Experience of Running SLAM
* Engineering Tricks
* Challenge


.. toctree::
   :maxdepth: 1
   :hidden:
   :caption: SLAM Basis

   slam_basis/index


.. toctree::
   :maxdepth: 1
   :hidden:
   :caption: Localization & Mapping

   slam_framworks/index
   slam_odometry/index
   slam_loopclosure/index
   slam_mapping/index


.. toctree::
   :maxdepth: 1
   :hidden:
   :caption: Multiple Sensor Fusion

   slam_msf/index


.. toctree::
   :maxdepth: 1
   :hidden:
   :caption: SLAM Benchmark

   slam_benchmark/index


.. toctree::
   :maxdepth: 1
   :hidden:
   :caption: SLAM Applications

   slam_modules/index


.. toctree::
   :maxdepth: 1
   :hidden:
   :caption: SLAM QA

   slam_qa/index
