// SPDX-License-Identifier: Apache-2.0
// 고정된 시작점과 종점 사이의 직선 경로 QP 해를 검사합니다.

#include "planning/jerk_qp_solver.hpp"
#include "planning/path_optimizer_bounds.hpp"
#include "planning/path_optimizer_qp.hpp"
#include "planning/path_optimizer_reference_data.hpp"

#include <cmath>
#include <stdexcept>
#include <vector>

namespace {
void check(const bool condition) {
  if (!condition) throw std::runtime_error("경로 최적화 QP 풀이 검사 실패");
}
}  // 네임스페이스

int main() {
  std::vector<planning::PathOptimizerReference> references(3);
  for (auto &reference : references) {
    reference.arc_length_to_next = 1.0;
    reference.beta = {0.0};
    reference.circle_bounds = {{-2.0, 2.0}};
  }
  references.front().fixed_state = Eigen::Vector2d(0.1, 0.0);
  references.back().fixed_state = Eigen::Vector2d(0.0, 0.0);
  planning::PathOptimizerQpParameters parameters;
  parameters.optimization_center_offset = 0.0;
  parameters.soft_constraints = false;
  parameters.hard_constraints = true;
  parameters.steering_limit_constraint = true;
  const auto qp = planning::buildPathOptimizerQp(
      references, {{0.0, 1.0}}, false, parameters);
  planning::JerkQpProblem solve_input;
  solve_input.hessian = qp.hessian;
  solve_input.linear_cost = qp.gradient;
  solve_input.constraints = qp.constraints;
  solve_input.lower_bound = qp.lower_bound;
  solve_input.upper_bound = qp.upper_bound;
  planning::JerkQpSolveOptions options;
  options.absolute_tolerance = 1e-7;
  options.relative_tolerance = 1e-7;
  const auto result = planning::solveJerkQp(solve_input, options);
  check(result.solved);
  const auto &x = result.solution;
  check(x.size() == 8);
  check(std::abs(x(0) - 0.1) < 1e-4);
  check(std::abs(x(1)) < 1e-4);
  check(std::abs(x(2) - 0.1) < 1e-4);
  check(std::abs(x(3) + 0.1) < 1e-4);
  check(std::abs(x(4)) < 1e-4);
  check(std::abs(x(5)) < 1e-4);
  check(std::abs(x(6) + 0.279) < 1e-4);
  check(std::abs(x(7) - 0.279) < 1e-4);
  const Eigen::VectorXd ax = qp.constraints * x;
  check((ax.array() >= qp.lower_bound.array() - 1e-4).all());
  check((ax.array() <= qp.upper_bound.array() + 1e-4).all());

  planning::PathOptimizerReferenceGeometry geometry;
  geometry.points.resize(5);
  for (std::size_t i = 0; i < 5; ++i)
    geometry.points[i].x = static_cast<double>(i);
  geometry.curvature.assign(5, 0.0);
  geometry.arc_length_to_next = {1.0, 1.0, 1.0, 1.0, 0.0};
  const std::vector<planning::PathOptimizerPoint2D> left{{-2.0, 3.0}, {6.0, 3.0}};
  const std::vector<planning::PathOptimizerPoint2D> right{{-2.0, -3.0}, {6.0, -3.0}};
  auto center_bounds = planning::pathOptimizerCenterBounds(
      geometry.points, left, right, 2.0, 0.0);
  planning::keepPathOptimizerMinimumBoundsWidth(
      center_bounds, geometry.curvature, 1.0, 2.0, 3.0);
  center_bounds = planning::extendPathOptimizerViolatedBounds(
      center_bounds, 1.0, 1.0);
  auto full_references = planning::buildPathOptimizerReferenceData(
      geometry, center_bounds);
  planning::attachPathOptimizerVehicleBounds(
      geometry, center_bounds, {{-1.0, 0.5}, {0.0, 0.5}, {1.0, 0.5}},
      full_references);
  full_references.front().fixed_state = Eigen::Vector2d::Zero();
  full_references.back().fixed_state = Eigen::Vector2d::Zero();
  const auto full_problem = planning::buildPathOptimizerQp(
      full_references, {{-1.0, 0.5}, {0.0, 0.5}, {1.0, 0.5}}, false);
  solve_input.hessian = full_problem.hessian;
  solve_input.linear_cost = full_problem.gradient;
  solve_input.constraints = full_problem.constraints;
  solve_input.lower_bound = full_problem.lower_bound;
  solve_input.upper_bound = full_problem.upper_bound;
  const auto full_solution = planning::solveJerkQp(solve_input, options);
  check(full_solution.solved);
  check(full_solution.solution.allFinite());
}
