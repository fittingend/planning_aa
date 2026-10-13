// SPDX-License-Identifier: Apache-2.0
// 독립 주차 노드의 활성화, 재계획, 구간 전환 및 완료 상태를 계산합니다.
#pragma once

#include "planning/behavior_pull_over_core.hpp"

#include <deque>
#include <functional>
#include <optional>

namespace planning {

struct FreespacePlannerCoreParameters {
  double arrived_distance_m = 0.5;
  double stopped_time_s = 1.0;
  double stopped_velocity_mps = 0.01;
  double course_out_distance_m = 1.0;
  double obstacle_time_s = 1.0;
  bool replan_when_obstacle_found = true;
  bool replan_when_course_out = true;
  FreespacePullOverParameters search = [] {
    FreespacePullOverParameters parameters;
    parameters.velocity_mps = 5.0 / 3.6;
    parameters.maximum_steering_angle_rad = 0.35;
    parameters.vehicle_shape_margin_m = 0.4;
    parameters.obstacle_threshold = 100;
    parameters.reverse_weight = 0.7;
    parameters.curve_weight = 0.5;
    parameters.smoothness_weight = 0.5;
    parameters.near_goal_distance_m = 3.0;
    parameters.goal_lateral_distance_weight = 5.0;
    parameters.adapt_expansion_distance = true;
    parameters.use_reeds_shepp_heuristic = true;
    parameters.use_source_obstacle_edt = true;
    parameters.append_exact_goal = false;
    parameters.source_goal_shift_policy = true;
    parameters.stop_at_phase_changes = false;
    parameters.direction_change_weight = 2.0;
    parameters.obstacle_distance_weight = 1.75;
    parameters.final_straight_distance_m = 0.0;
    return parameters;
  }();
};

struct FreespacePlannerCoreInput {
  double now_s = 0.0;
  bool parking_active = false;
  std::optional<TrajectoryPoint> ego;
  std::optional<CrosswalkOccupancyGrid> occupancy_grid;
};

struct FreespacePlannerCoreOutput {
  std::optional<Trajectory> trajectory;
  std::optional<Trajectory> full_trajectory;
  std::optional<bool> completed_status;
  bool waiting_for_stop = false;
  bool planned = false;
  bool completed = false;
  std::size_t target_index = 0;
};

class FreespacePlannerCore {
 public:
  using Search = std::function<std::optional<ShiftPullOverCandidate>(
      const TrajectoryPoint &, BehaviorGoalPose,
      const CrosswalkOccupancyGrid &, const FreespacePullOverParameters &)>;

  explicit FreespacePlannerCore(
      FreespacePlannerCoreParameters parameters = {}, Search search = {});

  void onRoute(BehaviorGoalPose goal);
  FreespacePlannerCoreOutput tick(const FreespacePlannerCoreInput &input);

 private:
  void reset(FreespacePlannerCoreOutput &output);
  bool stopped() const;
  bool obstacleOnPartial(const CrosswalkOccupancyGrid &grid,
                         const TrajectoryPoint &ego) const;
  bool courseOut(const TrajectoryPoint &ego) const;
  bool nearTarget(const TrajectoryPoint &ego) const;
  Trajectory partial() const;

  FreespacePlannerCoreParameters parameters_;
  Search search_;
  std::optional<BehaviorGoalPose> goal_;
  Trajectory trajectory_;
  Trajectory partial_trajectory_;
  std::vector<std::size_t> reversing_indices_;
  std::size_t previous_target_index_ = 0;
  std::size_t target_index_ = 0;
  std::deque<std::pair<double, double>> odometry_;
  std::optional<double> obstacle_found_at_s_;
  bool new_parking_cycle_ = false;
  bool reset_in_progress_ = false;
  bool completed_ = false;
  bool pending_reset_status_ = false;
};

}  // namespace planning
