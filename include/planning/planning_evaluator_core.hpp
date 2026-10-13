// SPDX-License-Identifier: Apache-2.0
// Planning evaluator의 기본 선택 지표를 ROS 없이 계산합니다.

#pragma once

#include "planning/behavior_goal_refinement.hpp"
#include "planning/trajectory.hpp"

#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace planning {

struct PlanningMetricStat {
  std::size_t count = 0;
  double minimum = 0.0;
  double maximum = 0.0;
  double mean = 0.0;
  void add(double value);
};

struct PlanningEvaluatorParameters {
  double minimum_relative_angle_point_distance_m = 0.1;
  double lookahead_distance_m = 5.0;
  double lookahead_time_s = 3.0;
  double evaluation_time_s = 5.0;
  double obstacle_collision_distance_m = 1.0;
};

struct PlanningEvaluatorInput {
  std::optional<TrajectoryPoint> ego;
  double ego_speed_mps = 0.0;
  std::optional<double> ego_acceleration_mps2;
  double acceleration_stamp_s = 0.0;
  std::optional<Trajectory> trajectory;
  std::optional<Trajectory> reference_trajectory;
  std::optional<std::vector<std::pair<double, double>>> object_centers;
  std::optional<BehaviorGoalPose> modified_goal;
  // 지도 어댑터가 route 내 최근접 차로를 공급할 때만 발행한다.
  std::optional<std::int64_t> ego_lane_id;
  std::optional<double> ego_lane_arc_m;
  std::optional<double> ego_lane_lateral_m;
};

struct PlanningEvaluatorResult {
  std::map<std::string, PlanningMetricStat> metrics;
  std::map<std::string, double> scalar_metrics;
};

class PlanningEvaluatorCore {
 public:
  explicit PlanningEvaluatorCore(PlanningEvaluatorParameters parameters = {});
  // 기준 노드는 10Hz timer에서 최신 polling 입력을 사용한다.
  PlanningEvaluatorResult tick(const PlanningEvaluatorInput &input);
  void reset();

 private:
  PlanningEvaluatorParameters parameters_;
  Trajectory previous_;
  Trajectory reference_;
  std::vector<std::pair<double, double>> object_centers_;
  std::optional<std::pair<double, double>> previous_acceleration_;
};

}  // namespace planning
