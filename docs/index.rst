.. SLAMKit documentation master file, created by
   sphinx-quickstart on Tue May 19 23:25:56 2020.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Welcome to SLAMKit's documentation!
====================================

* `Source Code (GitHub) <https://github.com/cggos/slamkit>`_

.. image:: imgs/slam.jpg


Study Notes
-------------------------

* `Simultaneous localization and mapping (Wikipedia) <https://en.wikipedia.org/wiki/Simultaneous_localization_and_mapping>`_

* `SLAM Blog (CSDN) <https://blog.csdn.net/u011178262/article/category/7456224/>`_

* SLAM Blog (CGABC)   
   * `vSLAM <https://cgabc.xyz/tags/Visual-SLAM/>`_
   * `LiDAR SLAM <https://cgabc.xyz/tags/LiDAR-SLAM/>`_

* `SLAM Notes (LaTeX on Overleaf) <https://www.overleaf.com/read/drmrxvnphrck/>`_


SLAM Basis
-------------------------

* Mathematics & Scientific Computing

  * https://sci.cgabc.xyz/

* Computer Vision

  * https://github.com/cggos/ccv
  * https://cgabc.xyz/tags/Computer-Vision/

* Kinematics and Dynamics

* State Estimation

  * https://github.com/cggos/state_estimation

* Sensors

  * Sensor Types

  * Sensor Calibration

    * https://cgabc.xyz/cl6ir5sdl006mgldj2ho4bojs/

* ROS

  * https://github.com/cggos/rokit



Localization & Mapping
-------------------------

* SLAM Frameworks

* SLAM Odometry

  * Visual Odometry/Visual Inertial Odometry
  * Laser Odometry
  * Wheel Odometry

* SLAM Loop Closure

  * Visual Vocabulary

* SLAM Mapping



GNSS & INS
-------------------------

* INS
* GNSS
* GIS
* Geomagnetics



Multiple Sensor Fusion
-------------------------



SLAM Benchmark
-------------------------

* SLAM Benchmark
* SLAM Dataset
* SLAM Simulation



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
   :caption: GNSS & INS

   slam_gnss_ins/index



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
