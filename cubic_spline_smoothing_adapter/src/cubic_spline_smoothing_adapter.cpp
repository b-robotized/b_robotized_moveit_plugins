/*********************************************************************
 * Software License Agreement (BSD License)
 * Data-Oriented Cubic Spline Trajectory Smoothing Adapter
 *********************************************************************/
#include <moveit/planning_interface/planning_response_adapter.hpp>
#include <class_loader/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <moveit/robot_trajectory/robot_trajectory.hpp>
#include <cubic_spline_smoothing_adapter/spline.h> 

#include <vector>
#include <cmath>
#include <algorithm>
#include <sstream>
#include <iomanip>

namespace cubic_spline_smoothing_adapter
{

namespace
{
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

// Extracted analytical solver to maintain pure function pipeline. 
// Evaluates bounds for physical hardware capabilities without discretization overhead.
std::vector<double> solveQuadraticInInterval(double A, double B, double C, double h)
{
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

SplineSegment buildSegment(double p0, double p1, double v0, double v1, double h)
{
  return {
    p0,
    v0,
    (3.0 * (p1 - p0) / (h * h)) - (2.0 * v0 + v1) / h,
    (2.0 * (p0 - p1) / (h * h * h)) + (v0 + v1) / (h * h),
    h
  };
}

PeakInfo analyzeSegmentPeaks(const SplineSegment& s)
{
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

// Splitting at zero-velocity seams isolates continuous motion profiles and mitigates 
// extreme C2 ringing across complete stops in industrial pick-and-place tasks.
std::vector<size_t> findSeams(const std::vector<std::vector<double>>& velocities, double seam_eps)
{
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

std::string formatLog(const std::vector<std::string>& names, const std::vector<double>& values)
{
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(4);
  for (size_t i = 0; i < names.size(); ++i) {
    oss << names[i] << ": " << values[i] << (i < names.size() - 1 ? ", " : "");
  }
  return oss.str();
}
} // anonymous namespace

class CubicSplineSmoothingAdapter : public planning_interface::PlanningResponseAdapter
{
public:
  CubicSplineSmoothingAdapter() : logger_(moveit::getLogger("moveit.ros.cubic_spline_smoothing_adapter")) {}
  ~CubicSplineSmoothingAdapter() override = default;

  void initialize(const rclcpp::Node::SharedPtr& node, const std::string& parameter_namespace) override
  {
    node->get_parameter_or(parameter_namespace + ".path_dev_max", config_.path_dev_max, 0.02);
    node->get_parameter_or(parameter_namespace + ".pos_tol", config_.pos_tol, 0.005);
    node->get_parameter_or(parameter_namespace + ".max_time_stretch", config_.max_time_stretch, 1.20);
    node->get_parameter_or(parameter_namespace + ".seam_eps", config_.seam_eps, 1e-6);
  }

  [[nodiscard]] std::string getDescription() const override { return "Cubic Spline Smoothing and Time Dilation Adapter"; }

  void adapt(
      const planning_scene::PlanningSceneConstPtr& /*planning_scene*/,
      const planning_interface::MotionPlanRequest& /*req*/,
      planning_interface::MotionPlanResponse& res) const override
  {
    if (res.error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS || !res.trajectory || res.trajectory->getWayPointCount() < 3) {
      return;
    }

    robot_trajectory::RobotTrajectoryPtr& traj = res.trajectory;
    const size_t num_waypoints = traj->getWayPointCount();
    const moveit::core::JointModelGroup* group = traj->getGroup();
    const std::vector<const moveit::core::JointModel*>& active_joints = group->getActiveJointModels();
    const size_t num_joints = active_joints.size();

    KinematicLimits limits;
    limits.v_max.reserve(num_joints);
    limits.a_max.reserve(num_joints);
    limits.a_min.reserve(num_joints);
    limits.p_min.reserve(num_joints);
    limits.p_max.reserve(num_joints);

    for (const auto* jm : active_joints) {
      const auto& bounds = jm->getVariableBounds();
      if (!bounds.empty()) {
        const auto& b = bounds[0];
        limits.v_max.push_back(b.velocity_bounded_ ? b.max_velocity_ : 1e6);
        limits.a_max.push_back(b.acceleration_bounded_ ? b.max_acceleration_ : 1e6);
        limits.a_min.push_back(b.acceleration_bounded_ ? b.min_acceleration_ : -limits.a_max.back());
        limits.p_min.push_back(b.position_bounded_ ? b.min_position_ : -1e6);
        limits.p_max.push_back(b.position_bounded_ ? b.max_position_ : 1e6);
      }
    }

    std::vector<double> times(num_waypoints);
    std::vector<std::vector<double>> positions(num_joints, std::vector<double>(num_waypoints));
    std::vector<std::vector<double>> velocities(num_joints, std::vector<double>(num_waypoints));
    
    std::vector<double> p_tmp(num_joints, 0.0);
    std::vector<double> v_tmp(num_joints, 0.0);

    for (size_t i = 0; i < num_waypoints; ++i) {
      times[i] = traj->getWayPointDurationFromStart(i);
      const moveit::core::RobotState& state = traj->getWayPoint(i);
      
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

    const std::vector<size_t> seam_indices = findSeams(velocities, config_.seam_eps);
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

    double k_req = 1.0;
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
          
          k_req = std::max({k_req, k_vel, k_acc, k_dec});
        }
      }
    }

    if (k_req > config_.max_time_stretch) {
      RCLCPP_ERROR(logger_, "Required time dilation (%f) exceeds max allowed (%f).", k_req, config_.max_time_stretch);
      res.error_code.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
      return;
    }

    if (k_req > 1.0001) {
      for (double& t : times) t *= k_req;
      for (auto& v_joint : velocities) {
        for (double& v : v_joint) v /= k_req;
      }
      fitAllSplines();
    }

    std::vector<double> log_v(num_joints, 0.0), log_a(num_joints, 0.0), log_d(num_joints, 0.0), log_dev(num_joints, 0.0);
    for (size_t s = 0; s < num_segments; ++s) {
      for (size_t i = seam_indices[s]; i < seam_indices[s + 1]; ++i) {
        double h = times[i+1] - times[i];
        for (size_t j = 0; j < num_joints; ++j) {
          double v0 = segment_splines[s][j].deriv(1, times[i]);
          double v1 = segment_splines[s][j].deriv(1, times[i+1]);
          
          SplineSegment seg = buildSegment(positions[j][i], positions[j][i+1], v0, v1, h);
          PeakInfo peaks = analyzeSegmentPeaks(seg);
          
          log_v[j] = std::max(log_v[j], peaks.max_vel);
          log_a[j] = std::max(log_a[j], peaks.max_acc_pos);
          log_d[j] = std::min(log_d[j], peaks.max_acc_neg);
          log_dev[j] = std::max(log_dev[j], peaks.max_dev);
          
          if (peaks.max_pos > limits.p_max[j] + config_.pos_tol || peaks.min_pos < limits.p_min[j] - config_.pos_tol) {
            RCLCPP_ERROR(logger_, "Position limit violated on joint %zu during smoothing.", j);
            res.error_code.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
            return;
          }
          
          if (peaks.max_dev > config_.path_dev_max) {
            RCLCPP_ERROR(logger_, "Path deviation limit violated on joint %zu during smoothing.", j);
            res.error_code.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
            return;
          }
        }
      }
    }

    for (size_t s = 0; s < num_segments; ++s) {
      for (size_t i = seam_indices[s]; i <= seam_indices[s + 1]; ++i) {
        moveit::core::RobotState* state = traj->getWayPointPtr(i).get();
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
      traj->setWayPointDurationFromPrevious(i, times[i] - times[i - 1]);
    }

    std::vector<std::string> j_names = group->getActiveJointModelNames();
    RCLCPP_INFO(logger_, "--- Spline Smoothing Complete ---");
    if (k_req > 1.0001) {
      RCLCPP_WARN(logger_, "Time dilated by: %.2f%%", (k_req - 1.0) * 100.0);
    }
    RCLCPP_INFO_STREAM(logger_, "Peak Vels [rad/s]:   " << formatLog(j_names, log_v));
    RCLCPP_INFO_STREAM(logger_, "Peak Accs [rad/s^2]: " << formatLog(j_names, log_a));
    RCLCPP_INFO_STREAM(logger_, "Peak Decs [rad/s^2]: " << formatLog(j_names, log_d));
    RCLCPP_INFO_STREAM(logger_, "Max Deviations [rad]: " << formatLog(j_names, log_dev));
  }

private:
  rclcpp::Logger logger_;
  SmoothingConfig config_;
};

} // namespace cubic_spline_smoothing_adapter

CLASS_LOADER_REGISTER_CLASS(cubic_spline_smoothing_adapter::CubicSplineSmoothingAdapter,
                            planning_interface::PlanningResponseAdapter)