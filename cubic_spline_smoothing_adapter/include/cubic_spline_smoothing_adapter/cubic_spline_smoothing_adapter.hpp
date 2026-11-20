// Copyright (c) 2026, b-robotized Group
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <string>

#include <moveit/planning_interface/planning_response_adapter.hpp>
#include <rclcpp/rclcpp.hpp>

#include "cubic_spline_smoothing_adapter/cubic_spline_smoothing.hpp"

namespace cubic_spline_smoothing_adapter
{

class CubicSplineSmoothingAdapter : public planning_interface::PlanningResponseAdapter
{
public:
  CubicSplineSmoothingAdapter();
  ~CubicSplineSmoothingAdapter() override = default;

  void initialize(
    const rclcpp::Node::SharedPtr & node, const std::string & parameter_namespace) override;

  [[nodiscard]] std::string getDescription() const override;

  void adapt(
    const planning_scene::PlanningSceneConstPtr & planning_scene,
    const planning_interface::MotionPlanRequest & req,
    planning_interface::MotionPlanResponse & res) const override;

private:
  rclcpp::Logger logger_;
  SmoothingConfig config_;
};

}  // namespace cubic_spline_smoothing_adapter
