// SPDX-License-Identifier: Apache-2.0
// Connect the selected scenario, final JerkFiltered trajectory and validator.
#pragma once

#include "planning/jerk_node_policy.hpp"
#include "planning/jerk_path_limits.hpp"
#include "planning/jerk_smoother.hpp"
#include "planning/planning_validator_core.hpp"
#include "planning/scenario_selector.hpp"
#include "planning/velocity_limit_selector.hpp"

#include <optional>
#include <string>
#include <utility>

namespace planning {

struct PlanningFinalFrame {
  double now_s = 0.0;
  double odometry_stamp_s = 0.0;
  std::string route_id;
  ScenarioPosition position;
  Trajectory lane_driving;
  std::optional<Trajectory> parking;
  TrajectoryPoint ego;
  double ego_speed_mps = 0.0;
  double ego_acceleration_mps2 = 0.0;
  double maximum_velocity_mps = 4.17;
  std::optional<double> node_maximum_velocity_mps;
  bool external_limit_active = true;
  bool external_limit_use_constraints = false;
  double external_min_acceleration_mps2 = -1.0;
  double external_max_jerk_mps3 = 1.0;
  double external_min_jerk_mps3 = -1.0;
  bool autonomous = false;
  bool control_enabled = false;
  bool parking_completed = false;
};

// Apply one newly selected internal limit or clear event to the next smoother
// frame. With no event, the smoother keeps its previous limit state.
void applySelectedVelocityLimit(
    PlanningFinalFrame &frame, const VelocityLimit &limit);

struct PlanningFinalResult {
  std::optional<ScenarioSelection> scenario;
  std::optional<Trajectory> selected;
  std::optional<Trajectory> smoothed;
  std::optional<ValidationOutcome> validation;
  bool smoother_solved = false;
};

class PlanningFinalPipelineCore {
 public:
  PlanningFinalPipelineCore(
      ScenarioSelectorParameters scenario_parameters = {},
      JerkSmootherParameters smoother_parameters = {},
      JerkNodeOutputParameters node_parameters = {},
      JerkPathLimitParameters path_limit_parameters = {},
      PrecheckThresholds validator_thresholds = {},
      InvalidTrajectoryPolicy invalid_policy =
          InvalidTrajectoryPolicy::UsePreviousResult);

  PlanningFinalResult onFrame(const PlanningFinalFrame &frame);
  void reset();

 private:
  ScenarioSelectorParameters scenario_parameters_;
  JerkSmootherParameters smoother_parameters_;
  JerkNodeOutputParameters node_parameters_;
  JerkPathLimitParameters path_limit_parameters_;
  PrecheckThresholds validator_thresholds_;
  InvalidTrajectoryPolicy invalid_policy_;
  ScenarioSelectorCore scenario_;
  PlanningValidatorCore validator_;
  std::optional<Trajectory> previous_smoother_internal_;
  std::optional<std::pair<double, double>> previous_projected_ego_;
  std::optional<double> external_velocity_mps_;
  double external_limit_distance_m_ = 0.0;
  double maximum_velocity_with_deceleration_mps_ = 0.0;
  std::string route_id_;
};

}  // namespace planning
