#include "cubic_spline_smoothing_adapter/cubic_spline_smoothing_adapter.hpp"
#include <class_loader/class_loader.hpp>
#include <moveit/robot_trajectory/robot_trajectory.hpp>
#include <sstream>
#include <iomanip>

namespace cubic_spline_smoothing_adapter {

namespace {
  std::string formatLog(const std::vector<std::string>& names, const std::vector<double>& values) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(4);
    for (size_t i = 0; i < names.size(); ++i) {
      oss << names[i] << ": " << values[i] << (i < names.size() - 1 ? ", " : "");
    }
    return oss.str();
  }
} // namespace

CubicSplineSmoothingAdapter::CubicSplineSmoothingAdapter()
  : logger_(moveit::getLogger("moveit.ros.cubic_spline_smoothing_adapter")) {}

void CubicSplineSmoothingAdapter::initialize(const rclcpp::Node::SharedPtr& node, const std::string& parameter_namespace) {
  node->get_parameter_or(parameter_namespace + ".path_dev_max", config_.path_dev_max, 0.02);
  node->get_parameter_or(parameter_namespace + ".pos_tol", config_.pos_tol, 0.005);
  node->get_parameter_or(parameter_namespace + ".max_time_stretch", config_.max_time_stretch, 1.20);
  node->get_parameter_or(parameter_namespace + ".seam_eps", config_.seam_eps, 1e-6);
}

std::string CubicSplineSmoothingAdapter::getDescription() const {
  return "Cubic Spline Smoothing and Time Dilation Adapter";
}

void CubicSplineSmoothingAdapter::adapt(
    const planning_scene::PlanningSceneConstPtr& /*planning_scene*/,
    const planning_interface::MotionPlanRequest& /*req*/,
    planning_interface::MotionPlanResponse& res) const 
{
  if (res.error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS || !res.trajectory || res.trajectory->getWayPointCount() < 3) {
    return;
  }

  robot_trajectory::RobotTrajectoryPtr& traj = res.trajectory;
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
    } else {
      limits.v_max.push_back(1e6);
      limits.a_max.push_back(1e6);
      limits.a_min.push_back(-1e6);
      limits.p_min.push_back(-1e6);
      limits.p_max.push_back(1e6);
    }
  }

  SmoothingResult result = processTrajectory(config_, limits, *traj);

  if (result.status != SmoothingStatus::SUCCESS) {
    res.error_code.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
    switch(result.status) {
      case SmoothingStatus::INVALID_INPUT:
        RCLCPP_ERROR(logger_, "Smoothing failed: Invalid input trajectory.");
        break;
      case SmoothingStatus::EXCEEDS_MAX_TIME_STRETCH:
        RCLCPP_ERROR(logger_, "Smoothing failed: %s", result.error_message.c_str());
        break;
      case SmoothingStatus::POSITION_LIMIT_VIOLATION:
        RCLCPP_ERROR(logger_, "Smoothing failed: Position limit violated.");
        break;
      case SmoothingStatus::PATH_DEVIATION_VIOLATION:
        RCLCPP_ERROR(logger_, "Smoothing failed: Path deviation limit violated.");
        break;
      default:
        RCLCPP_ERROR(logger_, "Smoothing failed: Unknown error.");
    }
    return;
  }

  std::vector<std::string> j_names = group->getActiveJointModelNames();
  RCLCPP_INFO(logger_, "--- Spline Smoothing Complete ---");

  for (size_t s = 0; s < result.segment_time_stretch_factors.size(); ++s) {
    if (result.segment_time_stretch_factors[s] > 1.0001) {
      RCLCPP_WARN(logger_, "Segment %zu time dilated by: %.2f%%", s, (result.segment_time_stretch_factors[s] - 1.0) * 100.0);
    }
  }

  RCLCPP_INFO_STREAM(logger_, "Peak Vels [rad/s]:   " << formatLog(j_names, result.peak_velocities));
  RCLCPP_INFO_STREAM(logger_, "Peak Accs [rad/s^2]: " << formatLog(j_names, result.peak_accelerations));
  RCLCPP_INFO_STREAM(logger_, "Peak Decs [rad/s^2]: " << formatLog(j_names, result.peak_decelerations));
  RCLCPP_INFO_STREAM(logger_, "Max Deviations [rad]: " << formatLog(j_names, result.max_deviations));
}

} // namespace cubic_spline_smoothing_adapter

CLASS_LOADER_REGISTER_CLASS(cubic_spline_smoothing_adapter::CubicSplineSmoothingAdapter,
                            planning_interface::PlanningResponseAdapter)