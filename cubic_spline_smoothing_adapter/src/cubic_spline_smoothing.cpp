#include "cubic_spline_smoothing_adapter/cubic_spline_smoothing.hpp"
#include <cubic_spline_smoothing_adapter/spline.h>
#include <cmath>
#include <algorithm>
#include <sstream>

namespace cubic_spline_smoothing_adapter {

struct SplineSegment {
  double a, b, c, d, h;
  inline double eval(double t) const { return a + b * t + c * t * t + d * t * t * t; }
  inline double deriv1(double t) const { return b + 2.0 * c * t + 3.0 * d * t * t; }
  inline double deriv2(double t) const { return 2.0 * c + 6.0 * d * t; }
};

struct PeakInfo {
  double max_vel = 0.0;
  double max_acc_pos = 0.0;
  double max_acc_neg = 0.0;
  double max_pos = -1e9;
  double min_pos = 1e9;
  double max_dev = 0.0;
};

std::vector<double> solveQuadraticInInterval(double A, double B, double C, double h) {
  std::vector<double> roots;
  if (std::abs(A) < 1e-9) {
    if (std::abs(B) > 1e-9) {
      double t = -C / B;
      if (t >= 0.0 && t <= h) roots.push_back(t);
    }
    return roots;
  }
  double delta = B * B - 4.0 * A * C;
  if (delta >= 0.0) {
    double sqrt_delta = std::sqrt(delta);
    double t1 = (-B + sqrt_delta) / (2.0 * A);
    double t2 = (-B - sqrt_delta) / (2.0 * A);
    if (t1 >= 0.0 && t1 <= h) roots.push_back(t1);
    if (t2 >= 0.0 && t2 <= h && std::abs(t1 - t2) > 1e-9) roots.push_back(t2);
  }
  return roots;
}

SplineSegment buildSegment(double p0, double p1, double v0, double v1, double h) {
  return {
    p0,
    v0,
    (3.0 * (p1 - p0) / (h * h)) - (2.0 * v0 + v1) / h,
    (2.0 * (p0 - p1) / (h * h * h)) + (v0 + v1) / (h * h),
    h
  };
}

PeakInfo analyzeSegmentPeaks(const SplineSegment& s) {
  PeakInfo peaks;
  double a0 = s.deriv2(0.0);
  double a1 = s.deriv2(s.h);
  peaks.max_acc_pos = std::max({0.0, a0, a1});
  peaks.max_acc_neg = std::min({0.0, a0, a1});
  peaks.max_vel = std::max(std::abs(s.deriv1(0.0)), std::abs(s.deriv1(s.h)));
  
  if (std::abs(s.d) > 1e-9) {
    double t_peak = -s.c / (3.0 * s.d);
    if (t_peak > 0.0 && t_peak < s.h) {
      peaks.max_vel = std::max(peaks.max_vel, std::abs(s.deriv1(t_peak)));
    }
  }
  
  peaks.max_pos = std::max(s.eval(0.0), s.eval(s.h));
  peaks.min_pos = std::min(s.eval(0.0), s.eval(s.h));
  auto pos_roots = solveQuadraticInInterval(3.0 * s.d, 2.0 * s.c, s.b, s.h);
  for (double t : pos_roots) {
    peaks.max_pos = std::max(peaks.max_pos, s.eval(t));
    peaks.min_pos = std::min(peaks.min_pos, s.eval(t));
  }
  
  double slope = (s.eval(s.h) - s.eval(0.0)) / s.h;
  peaks.max_dev = 0.0;
  auto dev_roots = solveQuadraticInInterval(3.0 * s.d, 2.0 * s.c, s.b - slope, s.h);
  for (double t : dev_roots) {
    double line_val = s.eval(0.0) + slope * t;
    peaks.max_dev = std::max(peaks.max_dev, std::abs(s.eval(t) - line_val));
  }
  return peaks;
}

std::vector<size_t> findSeams(const std::vector<std::vector<double>>& velocities, double seam_eps) {
  std::vector<size_t> seams = {0};
  if (velocities.empty() || velocities[0].empty()) return seams;
  const size_t num_joints = velocities.size();
  const size_t num_waypoints = velocities[0].size();
  
  for (size_t i = 1; i < num_waypoints - 1; ++i) {
    bool is_zero_vel = true;
    for (size_t j = 0; j < num_joints; ++j) {
      if (std::abs(velocities[j][i]) > seam_eps) {
        is_zero_vel = false;
        break;
      }
    }
    if (is_zero_vel) seams.push_back(i);
  }
  seams.push_back(num_waypoints - 1);
  return seams;
}

SmoothingResult processTrajectory(
    const SmoothingConfig& config,
    const KinematicLimits& limits,
    robot_trajectory::RobotTrajectory& traj) 
{
  const moveit::core::JointModelGroup* group = traj.getGroup();
  const size_t num_joints = group->getActiveJointModels().size();

  SmoothingResult result;
  result.peak_velocities.assign(num_joints, 0.0);
  result.peak_accelerations.assign(num_joints, 0.0);
  result.peak_decelerations.assign(num_joints, 0.0);
  result.max_deviations.assign(num_joints, 0.0);

  const size_t num_waypoints = traj.getWayPointCount();
  
  if (num_waypoints < 3) {
    result.status = SmoothingStatus::INVALID_INPUT;
    return result;
  }

  std::vector<double> times(num_waypoints);
  std::vector<std::vector<double>> positions(num_joints, std::vector<double>(num_waypoints));
  std::vector<std::vector<double>> velocities(num_joints, std::vector<double>(num_waypoints));
  
  std::vector<double> p_tmp(num_joints, 0.0);
  std::vector<double> v_tmp(num_joints, 0.0);

  for (size_t i = 0; i < num_waypoints; ++i) {
    times[i] = traj.getWayPointDurationFromStart(i);
    const moveit::core::RobotState& state = traj.getWayPoint(i);
    
    state.copyJointGroupPositions(group, p_tmp);
    if (state.hasVelocities()) {
      state.copyJointGroupVelocities(group, v_tmp);
    } else {
      std::fill(v_tmp.begin(), v_tmp.end(), 0.0);
    }
    for (size_t j = 0; j < num_joints; ++j) {
      positions[j][i] = p_tmp[j];
      velocities[j][i] = v_tmp[j];
    }
  }

  const std::vector<size_t> seam_indices = findSeams(velocities, config.seam_eps);
  const size_t num_segments = seam_indices.size() - 1;
  std::vector<std::vector<tk::spline>> segment_splines(num_segments, std::vector<tk::spline>(num_joints));

  auto fitAllSplines = [&]() {
    for (size_t s = 0; s < num_segments; ++s) {
      size_t start_idx = seam_indices[s];
      size_t count = seam_indices[s + 1] - start_idx + 1;
      std::vector<double> t_seg(times.begin() + start_idx, times.begin() + start_idx + count);
      for (size_t j = 0; j < num_joints; ++j) {
        std::vector<double> p_seg(positions[j].begin() + start_idx, positions[j].begin() + start_idx + count);
        double v0 = velocities[j][start_idx];
        double v1 = velocities[j][seam_indices[s + 1]];
        segment_splines[s][j] = tk::spline();
        segment_splines[s][j].set_boundary(tk::spline::first_deriv, v0, tk::spline::first_deriv, v1);
        segment_splines[s][j].set_points(t_seg, p_seg);
      }
    }
  };

  fitAllSplines();
  result.segment_time_stretch_factors.assign(num_segments, 1.0);

  // Calculate required stretch per segment
  for (size_t s = 0; s < num_segments; ++s) {
    for (size_t i = seam_indices[s]; i < seam_indices[s + 1]; ++i) {
      double h = times[i+1] - times[i];
      for (size_t j = 0; j < num_joints; ++j) {
        double v0 = segment_splines[s][j].deriv(1, times[i]);
        double v1 = segment_splines[s][j].deriv(1, times[i+1]);
        SplineSegment seg = buildSegment(positions[j][i], positions[j][i+1], v0, v1, h);
        PeakInfo peaks = analyzeSegmentPeaks(seg);
        
        double k_vel = peaks.max_vel / limits.v_max[j];
        double k_acc = std::sqrt(std::abs(peaks.max_acc_pos) / limits.a_max[j]);
        double k_dec = std::sqrt(std::abs(peaks.max_acc_neg) / std::abs(limits.a_min[j]));
        result.segment_time_stretch_factors[s] = std::max({result.segment_time_stretch_factors[s], k_vel, k_acc, k_dec});
      }
    }
    if (result.segment_time_stretch_factors[s] > config.max_time_stretch) {
      result.status = SmoothingStatus::EXCEEDS_MAX_TIME_STRETCH;
      result.error_message = "Segment " + std::to_string(s) + " required time dilation exceeds max allowed.";
      return result;
    }
  }

  bool needs_time_stretch = false;
  double current_time = 0.0;
  std::vector<double> original_times = times;
  
  for (size_t s = 0; s < num_segments; ++s) {
    double k = result.segment_time_stretch_factors[s];
    if (k > 1.0001) needs_time_stretch = true;
    for (size_t i = seam_indices[s]; i < seam_indices[s + 1]; ++i) {
      times[i] = current_time;
      current_time += (original_times[i + 1] - original_times[i]) * k;
      if (k > 1.0001) {
        for (size_t j = 0; j < num_joints; ++j) velocities[j][i] /= k;
      }
    }
  }
  
  times.back() = current_time;
  if (result.segment_time_stretch_factors.back() > 1.0001) {
    for (size_t j = 0; j < num_joints; ++j) velocities[j].back() /= result.segment_time_stretch_factors.back();
  }

  if (needs_time_stretch) fitAllSplines();

  // Geometric validation
  for (size_t s = 0; s < num_segments; ++s) {
    for (size_t i = seam_indices[s]; i < seam_indices[s + 1]; ++i) {
      double h = times[i+1] - times[i];
      for (size_t j = 0; j < num_joints; ++j) {
        double v0 = segment_splines[s][j].deriv(1, times[i]);
        double v1 = segment_splines[s][j].deriv(1, times[i+1]);
        SplineSegment seg = buildSegment(positions[j][i], positions[j][i+1], v0, v1, h);
        PeakInfo peaks = analyzeSegmentPeaks(seg);
        
        result.peak_velocities[j] = std::max(result.peak_velocities[j], peaks.max_vel);
        result.peak_accelerations[j] = std::max(result.peak_accelerations[j], peaks.max_acc_pos);
        result.peak_decelerations[j] = std::min(result.peak_decelerations[j], peaks.max_acc_neg);
        result.max_deviations[j] = std::max(result.max_deviations[j], peaks.max_dev);
        
        if (peaks.max_pos > limits.p_max[j] + config.pos_tol || peaks.min_pos < limits.p_min[j] - config.pos_tol) {
          result.status = SmoothingStatus::POSITION_LIMIT_VIOLATION;
          return result;
        }
        if (peaks.max_dev > config.path_dev_max) {
          result.status = SmoothingStatus::PATH_DEVIATION_VIOLATION;
          return result;
        }
      }
    }
  }

  // Write variables back to RobotTrajectory
  for (size_t s = 0; s < num_segments; ++s) {
    for (size_t i = seam_indices[s]; i <= seam_indices[s + 1]; ++i) {
      moveit::core::RobotState* state = traj.getWayPointPtr(i).get();
      std::vector<double> smooth_vel(num_joints);
      std::vector<double> smooth_acc(num_joints);
      for (size_t j = 0; j < num_joints; ++j) {
        smooth_vel[j] = segment_splines[s][j].deriv(1, times[i]);
        smooth_acc[j] = segment_splines[s][j].deriv(2, times[i]);
      }
      state->setJointGroupVelocities(group, smooth_vel);
      state->setJointGroupAccelerations(group, smooth_acc);
    }
  }

  for (size_t i = 1; i < num_waypoints; ++i) {
    traj.setWayPointDurationFromPrevious(i, times[i] - times[i - 1]);
  }

  result.status = SmoothingStatus::SUCCESS;
  return result;
}

} // namespace cubic_spline_smoothing_adapter