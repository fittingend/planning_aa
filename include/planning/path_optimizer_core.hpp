// SPDX-License-Identifier: Apache-2.0
// ROS 없이 경로 최적화의 기준점 생성부터 최종 궤적까지 실행합니다.

#pragma once

#include "planning/jerk_qp_solver.hpp"
#include "planning/path_optimizer_bounds.hpp"
#include "planning/path_optimizer_reference_data.hpp"
#include "planning/path_optimizer_solution.hpp"

#include <optional>

namespace planning {

struct PathOptimizerCoreParameters {
  PathOptimizerReferenceGeometryParameters geometry;
  PathOptimizerAvoidanceParameters avoidance;
  PathOptimizerQpParameters qp;
  PathOptimizerValidation validation;
  PathOptimizerPostprocessParameters postprocess;
  JerkQpSolveOptions solver;
  double vehicle_length = 4.89;
  double rear_overhang = 1.10;
  double left_overhang = 0.128;
  double right_overhang = 0.128;
  double minimum_drivable_width = 0.2;
  double soft_road_clearance = 0.1;
  double maximum_longitudinal_margin_for_bound_violation = 1.0;
  double maximum_bound_fixing_time = 1.0;
  double ego_nearest_distance = 3.0;
  double ego_nearest_yaw = 1.046;
};

struct PathOptimizerCoreInput {
  Trajectory path;
  std::vector<PathOptimizerPoint2D> left_bound;
  std::vector<PathOptimizerPoint2D> right_bound;
  TrajectoryPoint ego;
  double ego_speed = 0.0;
  bool goal_contained = false;
};

struct PathOptimizerResetParameters {
  double maximum_lateral_path_change = 2.0;
  double maximum_ego_movement = 5.0;
  double maximum_stopped_goal_movement = 15.0;
};

// 원본 재계획 검사기의 이전 QP 상태 초기화 조건을 계산합니다.
bool requiresPathOptimizerReset(
    const PathOptimizerCoreInput &current,
    const std::optional<PathOptimizerCoreInput> &previous,
    const PathOptimizerResetParameters &parameters = {});

struct PathOptimizerCoreResult {
  PathOptimizerReferenceGeometry geometry;
  std::vector<PathOptimizerBound> center_bounds;
  std::vector<PathOptimizerReference> references;
  Trajectory optimized;
  Trajectory output;
  JerkQpSolveResult solve;
};

class PathOptimizerCore {
 public:
  explicit PathOptimizerCore(PathOptimizerCoreParameters parameters = {});
  std::optional<PathOptimizerCoreResult> optimize(const PathOptimizerCoreInput &input);
  void reset();

 private:
  PathOptimizerCoreParameters parameters_;
  std::optional<PathOptimizerReferenceGeometry> previous_geometry_;
  std::vector<PathOptimizerBound> previous_bounds_;
  std::vector<PathOptimizerReference> previous_references_;
};

}  // planning 네임스페이스
