#pragma once

#include <moveit/planning_interface/planning_response_adapter.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "cubic_spline_smoothing_adapter/cubic_spline_smoothing.hpp"

namespace cubic_spline_smoothing_adapter {

class CubicSplineSmoothingAdapter : public planning_interface::PlanningResponseAdapter {
public:
  CubicSplineSmoothingAdapter();
  ~CubicSplineSmoothingAdapter() override = default;

  void initialize(const rclcpp::Node::SharedPtr& node, const std::string& parameter_namespace) override;
  
  [[nodiscard]] std::string getDescription() const override;
  
  void adapt(const planning_scene::PlanningSceneConstPtr& planning_scene,
             const planning_interface::MotionPlanRequest& req,
             planning_interface::MotionPlanResponse& res) const override;

private:
  rclcpp::Logger logger_;
  SmoothingConfig config_;
};

} // namespace cubic_spline_smoothing_adapter