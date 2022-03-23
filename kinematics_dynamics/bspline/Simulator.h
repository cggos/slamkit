/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2021 Patrick Geneva
 * Copyright (C) 2021 Guoquan Huang
 * Copyright (C) 2021 OpenVINS Contributors
 * Copyright (C) 2019 Kevin Eckenhoff
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef OV_MSCKF_SIMULATOR_H
#define OV_MSCKF_SIMULATOR_H

#include <Eigen/Eigen>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <random>
#include <sstream>
#include <string>
#include <unordered_map>

#include "BsplineSE3.h"
#include "utils/colors.h"
#include "utils/dataset_reader.h"

namespace ov_msckf {

/**
 * @brief Struct of our imu noise parameters
 */
struct NoiseManager {
  /// Gyroscope white noise (rad/s/sqrt(hz))
  double sigma_w = 1.6968e-04;

  /// Gyroscope white noise covariance
  double sigma_w_2 = pow(1.6968e-04, 2);

  /// Gyroscope random walk (rad/s^2/sqrt(hz))
  double sigma_wb = 1.9393e-05;

  /// Gyroscope random walk covariance
  double sigma_wb_2 = pow(1.9393e-05, 2);

  /// Accelerometer white noise (m/s^2/sqrt(hz))
  double sigma_a = 2.0000e-3;

  /// Accelerometer white noise covariance
  double sigma_a_2 = pow(2.0000e-3, 2);

  /// Accelerometer random walk (m/s^3/sqrt(hz))
  double sigma_ab = 3.0000e-03;

  /// Accelerometer random walk covariance
  double sigma_ab_2 = pow(3.0000e-03, 2);

  /// Nice print function of what parameters we have loaded
  void print() {
    PRINT_DEBUG("  - gyroscope_noise_density: %.6f\n", sigma_w);
    PRINT_DEBUG("  - accelerometer_noise_density: %.5f\n", sigma_a);
    PRINT_DEBUG("  - gyroscope_random_walk: %.7f\n", sigma_wb);
    PRINT_DEBUG("  - accelerometer_random_walk: %.6f\n", sigma_ab);
  }
};

struct SimParam {
  int num_cameras = 0;

  /// Gravity magnitude in the global frame (i.e. should be 9.81 typically)
  double gravity_mag = 9.81;

  /// Time offset between camera and IMU.
  double calib_camimu_dt = 0.0;

  /// The number of points we should extract and track in *each* image frame. This highly effects the computation
  /// required for tracking.
  int num_pts = 150;

  /// Noise sigma for our raw pixel measurements
  double sigma_pix = 1;

  bool use_stereo = false;

  NoiseManager imu_noises;

  /// Seed for initial states (i.e. random feature 3d positions in the generated map)
  int sim_seed_state_init = 0;

  /// Seed for calibration perturbations. Change this to perturb by different random values if perturbations are
  /// enabled.
  int sim_seed_preturb = 0;

  /// Measurement noise seed. This should be incremented for each run in the Monte-Carlo simulation to generate the same
  /// true measurements, but diffferent noise values.
  int sim_seed_measurements = 0;

  /// If we should perturb the calibration that the estimator starts with
  bool sim_do_perturbation = false;

  /// Path to the trajectory we will b-spline and simulate on. Should be time(s),pos(xyz),ori(xyzw) format.
  std::string sim_traj_path = "src/open_vins/ov_data/sim/udel_gore.txt";

  /// We will start simulating after we have moved this much along the b-spline. This prevents static starts as we init
  /// from groundtruth in simulation.
  double sim_distance_threshold = 1.2;

  /// Frequency (Hz) that we will simulate our cameras
  double sim_freq_cam = 10.0;

  /// Frequency (Hz) that we will simulate our inertial measurement unit
  double sim_freq_imu = 400.0;

  /// Feature distance we generate features from (minimum)
  double sim_min_feature_gen_distance = 5;

  /// Feature distance we generate features from (maximum)
  double sim_max_feature_gen_distance = 10;
};

/**
 * @brief Master simulator class that generated visual-inertial measurements
 *
 * Given a trajectory this will generate a SE(3) @ref ov_core::BsplineSE3 for that trajectory.
 * This allows us to get the inertial measurement information at each timestep during this trajectory.
 * After creating the bspline we will generate an environmental feature map which will be used as our feature
 * measurements. This map will be projected into the frame at each timestep to get our "raw" uv measurements. We inject
 * bias and white noises into our inertial readings while adding our white noise to the uv measurements also. The user
 * should specify the sensor rates that they desire along with the seeds of the random number generators.
 *
 */
class Simulator {
 public:
  /**
   * @brief Default constructor, will load all configuration variables
   * @param params_ VioManager parameters. Should have already been loaded from cmd.
   */
  Simulator(SimParam &params_);

  /**
   * @brief Will get a set of perturbed parameters
   * @param gen_state Random number gen to use
   * @param params_ Parameters we will perturb
   */
  // static void perturb_parameters(std::mt19937 gen_state, SimParam &params_);

  /**
   * @brief Returns if we are actively simulating
   * @return True if we still have simulation data
   */
  bool ok() { return is_running; }

  /**
   * @brief Gets the timestamp we have simulated up too
   * @return Timestamp
   */
  double current_timestamp() { return timestamp; }

  /**
   * @brief Get the simulation state at a specified timestep
   * @param desired_time Timestamp we want to get the state at
   * @param imustate State in the MSCKF ordering: [time(sec),q_GtoI,p_IinG,v_IinG,b_gyro,b_accel]
   * @return True if we have a state
   */
  bool get_state(double desired_time, Eigen::Matrix<double, 17, 1> &imustate);

  /**
   * @brief Gets the next inertial reading if we have one.
   * @param time_imu Time that this measurement occured at
   * @param wm Angular velocity measurement in the inertial frame
   * @param am Linear velocity in the inertial frame
   * @return True if we have a measurement
   */
  bool get_next_imu(double &time_imu, Eigen::Vector3d &wm, Eigen::Vector3d &am);

  /**
   * @brief Gets the next inertial reading if we have one.
   * @param time_cam Time that this measurement occured at
   * @param camids Camera ids that the corresponding vectors match
   * @param feats Noisy uv measurements and ids for the returned time
   * @return True if we have a measurement
   */
  bool get_next_cam(double &time_cam,
                    std::vector<int> &camids,
                    std::vector<std::vector<std::pair<size_t, Eigen::VectorXf>>> &feats);

  /// Returns the true 3d map of features
  std::unordered_map<size_t, Eigen::Vector3d> get_map() { return featmap; }

  /// Returns the true 3d map of features
  std::vector<Eigen::Vector3d> get_map_vec() {
    std::vector<Eigen::Vector3d> feats;
    for (auto const &feat : featmap) feats.push_back(feat.second);
    return feats;
  }

  /// Access function to get the true parameters (i.e. calibration and settings)
  SimParam get_true_parameters() { return params; }

 protected:
#if 0 
  /**
   * @brief Projects the passed map features into the desired camera frame.
   * @param R_GtoI Orientation of the IMU pose
   * @param p_IinG Position of the IMU pose
   * @param camid Camera id of the camera sensor we want to project into
   * @param feats Our set of 3d features
   * @return True distorted raw image measurements and their ids for the specified camera
   */
  std::vector<std::pair<size_t, Eigen::VectorXf>> project_pointcloud(
      const Eigen::Matrix3d &R_GtoI,
      const Eigen::Vector3d &p_IinG,
      int camid,
      const std::unordered_map<size_t, Eigen::Vector3d> &feats);

  /**
   * @brief Will generate points in the fov of the specified camera
   * @param R_GtoI Orientation of the IMU pose
   * @param p_IinG Position of the IMU pose
   * @param camid Camera id of the camera sensor we want to project into
   * @param[out] feats Map we will append new features to
   * @param numpts Number of points we should generate
   */
  void generate_points(const Eigen::Matrix3d &R_GtoI,
                       const Eigen::Vector3d &p_IinG,
                       int camid,
                       std::unordered_map<size_t, Eigen::Vector3d> &feats,
                       int numpts);
#endif

  //===================================================================
  // Configuration variables
  //===================================================================

  /// True vio manager params (a copy of the parsed ones)
  SimParam params;

  //===================================================================
  // State related variables
  //===================================================================

  /// Our loaded trajectory data (timestamp(s), q_GtoI, p_IinG)
  std::vector<Eigen::VectorXd> traj_data;

  /// Our b-spline trajectory
  ov_core::BsplineSE3 spline;

  /// Our map of 3d features
  size_t id_map = 0;
  std::unordered_map<size_t, Eigen::Vector3d> featmap;

  /// Mersenne twister PRNG for measurements (IMU)
  std::mt19937 gen_meas_imu;

  /// Mersenne twister PRNG for measurements (CAMERAS)
  std::vector<std::mt19937> gen_meas_cams;

  /// Mersenne twister PRNG for state initialization
  std::mt19937 gen_state_init;

  /// Mersenne twister PRNG for state perturbations
  std::mt19937 gen_state_perturb;

  /// If our simulation is running
  bool is_running;

  //===================================================================
  // Simulation specific variables
  //===================================================================

  /// Current timestamp of the system
  double timestamp;

  /// Last time we had an IMU reading
  double timestamp_last_imu;

  /// Last time we had an CAMERA reading
  double timestamp_last_cam;

  /// Our running acceleration bias
  Eigen::Vector3d true_bias_accel = Eigen::Vector3d::Zero();

  /// Our running gyroscope bias
  Eigen::Vector3d true_bias_gyro = Eigen::Vector3d::Zero();

  // Our history of true biases
  std::vector<double> hist_true_bias_time;
  std::vector<Eigen::Vector3d> hist_true_bias_accel;
  std::vector<Eigen::Vector3d> hist_true_bias_gyro;
};

}  // namespace ov_msckf

#endif  // OV_MSCKF_SIMULATOR_H
