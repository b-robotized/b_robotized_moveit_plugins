#pragma once

#include <vector>
#include <string>
#include <cstddef>
#include <moveit/robot_trajectory/robot_trajectory.hpp>

namespace cubic_spline_smoothing_adapter {

struct SmoothingConfig {
  double path_dev_max = 0.02;
  double pos_tol = 0.005;
  double max_time_stretch = 1.20;
  double seam_eps = 1e-6;
};

struct KinematicLimits {
  std::vector<double> v_max;
  std::vector<double> a_max;
  std::vector<double> a_min;
  std::vector<double> p_min;
  std::vector<double> p_max;
};

enum class SmoothingStatus {
  SUCCESS,
  INVALID_INPUT,
  EXCEEDS_MAX_TIME_STRETCH,
  POSITION_LIMIT_VIOLATION,
  PATH_DEVIATION_VIOLATION
};

struct SmoothingResult {
  SmoothingStatus status;
  std::string error_message;
  
  std::vector<double> segment_time_stretch_factors;
  std::vector<double> peak_velocities;
  std::vector<double> peak_accelerations;
  std::vector<double> peak_decelerations;
  std::vector<double> max_deviations;
};

// Modifies the trajectory directly using C^2 cubic splines
SmoothingResult processTrajectory(
    const SmoothingConfig& config,
    const KinematicLimits& limits,
    robot_trajectory::RobotTrajectory& traj);

std::vector<size_t> findSeams(const std::vector<std::vector<double>>& velocities, double seam_eps);

} // namespace cubic_spline_smoothing_adapter