// SPDX-License-Identifier: Apache-2.0
// 경로 최적화 계산 단위를 순서대로 연결하는 독립 실행 코어입니다.

#include "planning/path_optimizer_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace planning {

PathOptimizerCore::PathOptimizerCore(PathOptimizerCoreParameters parameters)
    : parameters_(std::move(parameters)) {}

void PathOptimizerCore::reset() {
  previous_geometry_.reset();
  previous_bounds_.clear();
  previous_references_.clear();
}

std::optional<PathOptimizerCoreResult> PathOptimizerCore::optimize(
    const PathOptimizerCoreInput &input) {
  if (input.path.size() < 2 || !std::isfinite(input.ego_speed)) {
    throw std::invalid_argument("경로 최적화 입력 경로 또는 차량 속도가 유효하지 않습니다");
  }
  PathOptimizerCoreResult result;
  const auto &p = parameters_;
  auto extended_geometry_parameters = p.geometry;
  extended_geometry_parameters.maximum_points =
      std::numeric_limits<std::size_t>::max();
  result.geometry = preparePathOptimizerReferenceGeometry(
      input.path, input.ego, extended_geometry_parameters,
      previous_geometry_ ? &*previous_geometry_ : nullptr);
  result.center_bounds = pathOptimizerCenterBounds(
      result.geometry.points, input.left_bound, input.right_bound,
      p.qp.vehicle_width, p.soft_road_clearance, p.geometry.interval);
  const double maximum_offset = std::max(
      std::abs(-p.rear_overhang),
      std::abs(p.vehicle_length - p.rear_overhang));
  keepPathOptimizerMinimumBoundsWidth(
      result.center_bounds, result.geometry.curvature,
      p.minimum_drivable_width, p.qp.vehicle_width, maximum_offset);
  result.center_bounds = extendPathOptimizerViolatedBounds(
      result.center_bounds,
      p.maximum_longitudinal_margin_for_bound_violation, p.geometry.interval);
  if (previous_geometry_ && !previous_bounds_.empty()) {
    reusePathOptimizerPreviousBounds(
        result.center_bounds, result.geometry, input.ego, input.ego_speed,
        *previous_geometry_, previous_bounds_, p.maximum_bound_fixing_time,
        p.geometry.interval, p.ego_nearest_distance,
        p.ego_nearest_yaw);
  }
  result.references = buildPathOptimizerReferenceData(
      result.geometry, result.center_bounds, p.avoidance,
      previous_geometry_ && !previous_references_.empty() ?
          &*previous_geometry_ : nullptr,
      previous_geometry_ && !previous_references_.empty() ?
          &previous_references_ : nullptr);
  const auto circles = fittingUniformCircles(
      p.vehicle_length, p.qp.vehicle_width, p.rear_overhang,
      p.left_overhang, p.right_overhang);
  attachPathOptimizerVehicleBounds(
      result.geometry, result.center_bounds, circles, result.references);
  if (result.geometry.points.size() > p.geometry.maximum_points) {
    result.geometry.points.resize(p.geometry.maximum_points);
    result.geometry.curvature.resize(p.geometry.maximum_points);
    result.geometry.arc_length_to_next.resize(p.geometry.maximum_points);
    result.geometry.fixed_states.resize(p.geometry.maximum_points);
    result.center_bounds.resize(p.geometry.maximum_points);
    result.references.resize(p.geometry.maximum_points);
  }
  const auto problem = buildPathOptimizerQp(
      result.references, circles, input.goal_contained, p.qp);
  JerkQpProblem solve_input;
  solve_input.hessian = problem.hessian;
  solve_input.linear_cost = problem.gradient;
  solve_input.constraints = problem.constraints;
  solve_input.lower_bound = problem.lower_bound;
  solve_input.upper_bound = problem.upper_bound;
  result.solve = solveJerkQp(solve_input, p.solver);
  if (!result.solve.solved) return std::nullopt;
  const auto optimized = pathOptimizerSolutionToTrajectory(
      result.geometry.points, problem, result.solve.solution, p.validation);
  if (!optimized) return std::nullopt;
  result.optimized = *optimized;
  result.output = finalizePathOptimizerTrajectory(
      input.path, result.optimized, input.ego, p.postprocess);
  storePathOptimizerSolutionStates(
      result.geometry, problem, result.solve.solution);
  previous_geometry_ = result.geometry;
  previous_bounds_ = result.center_bounds;
  previous_references_ = result.references;
  return result;
}

}  // planning 네임스페이스
