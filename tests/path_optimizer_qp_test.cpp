// SPDX-License-Identifier: Apache-2.0
// 경로 최적화 비용과 제약 행렬의 크기·계수·경계값을 검사합니다.

#include "planning/path_optimizer_qp.hpp"

#include <cmath>
#include <stdexcept>
#include <vector>

namespace {
void check(const bool condition) {
  if (!condition) throw std::runtime_error("경로 최적화 QP 검사 실패");
}
}  // 네임스페이스

int main() {
  const auto circles = planning::fittingUniformCircles(4.0, 2.0, 1.0, 0.2, 0.2);
  check(circles.size() == 3);
  check(circles[0].longitudinal_offset == -1.0);
  check(circles[2].longitudinal_offset == 3.0);
  check(circles[1].radius == 1.0);

  std::vector<planning::PathOptimizerReference> references(3);
  for (auto &reference : references) {
    reference.arc_length_to_next = 1.0;
    reference.beta = {0.0};
    reference.circle_bounds = {{-2.0, 2.0}};
  }
  references[0].fixed_state = Eigen::Vector2d(0.0, 0.0);
  references[1].avoidance_cost = 0.5;
  references[2].alpha = 0.1;
  planning::PathOptimizerQpParameters p;
  p.vehicle_width = 2.0;
  p.maximum_steering = 0.6;
  p.optimization_center_offset = 0.0;
  const auto problem = planning::buildPathOptimizerQp(
      references, {{0.0, 1.0}}, false, p);
  check(problem.state_variables == 6);
  check(problem.steering_variables == 2);
  check(problem.slack_variables == 3);
  check(problem.hessian.rows() == 11);
  check(problem.constraints.rows() == 17);
  check(std::abs(problem.hessian(0, 0) - 1.0) < 1e-12);
  check(std::abs(problem.hessian(2, 2) - 0.5) < 1e-12);
  check(std::abs(problem.hessian(3, 3) - 5.0) < 1e-12);
  check(std::abs(problem.hessian(4, 4) - 100.0 * std::cos(0.1) * std::cos(0.1)) < 1e-12);
  check(std::abs(problem.hessian(6, 6) - 2.0) < 1e-12);
  check(std::abs(problem.hessian(7, 7) - 51.5) < 1e-12);
  check(std::abs(problem.hessian(6, 7) + 1.0) < 1e-12);
  check(problem.gradient.tail(3).isOnes());
  check(problem.constraints(2, 1) == -1.0);
  check(std::abs(problem.constraints(3, 6) + 1.0 / p.wheelbase) < 1e-12);
  check(problem.constraints(6, 0) == 1.0);
  check(problem.constraints(6, 8) == 1.0);
  check(problem.lower_bound(6) == -2.0);
  check(problem.constraints(9, 0) == -1.0);
  check(problem.lower_bound(9) == -2.0);
  check(problem.constraints(12, 8) == 1.0);
  check(problem.lower_bound(12) == 0.0);
  check(problem.constraints(15, 0) == 1.0);
  check(problem.constraints(16, 1) == 1.0);
  check(problem.lower_bound(15) == 0.0);
  check(problem.upper_bound(16) == 0.0);

  auto limited = p;
  limited.steering_limit_constraint = true;
  limited.hard_constraints = true;
  limited.soft_constraints = false;
  references[0].curvature = 0.1;
  const auto hard = planning::buildPathOptimizerQp(
      references, {{0.0, 1.0}}, true, limited);
  check(hard.slack_variables == 0);
  check(hard.constraints.rows() == 13);
  check(hard.hessian.rows() == 8);
  check(std::abs(hard.hessian(4, 4) - 1000.0 * std::cos(0.1) * std::cos(0.1)) < 1e-12);
  check(hard.constraints(6, 0) == 1.0);
  check(hard.lower_bound(6) == -2.0);
  check(hard.upper_bound(6) == 2.0);
  check(hard.constraints(11, 6) == 1.0);
  const double reference_steering = std::atan2(p.wheelbase * 0.1, 1.0);
  check(std::abs(hard.lower_bound(11) -
                 (reference_steering - p.maximum_steering)) < 1e-12);

  auto shifted = p;
  shifted.optimization_center_offset = 2.0;
  references[0].beta = {0.2, -0.3};
  references[0].circle_bounds = {{-1.0, 2.0}, {-0.5, 1.5}};
  for (std::size_t i = 1; i < references.size(); ++i) {
    references[i].beta = {0.0, 0.0};
    references[i].circle_bounds = {{-2.0, 2.0}, {-2.0, 2.0}};
  }
  const std::vector<planning::PathOptimizerCircle> two_circles{
      {1.5, 1.2}, {-0.5, 0.9}};
  const auto shared = planning::buildPathOptimizerQp(
      references, two_circles, false, shifted);
  check(shared.slack_variables == 3);
  check(shared.constraints.rows() == 26);
  check(shared.constraints(6, 8) == 1.0);
  check(shared.constraints(15, 8) == 1.0);
  check(std::abs(shared.constraints(6, 0) - std::cos(0.2)) < 1e-12);
  check(std::abs(shared.constraints(6, 1) - 1.5 * std::cos(0.2)) < 1e-12);
  check(std::abs(shared.lower_bound(6) -
                 (-1.0 + 0.2 - 1.5 * std::sin(0.2))) < 1e-12);
  check(std::abs(shared.hessian(4, 5) -
                 100.0 * std::cos(0.1) * 2.0 * std::cos(0.1)) < 1e-12);
  check(std::abs(shared.gradient(4) -
                 100.0 * std::cos(0.1) * (-2.0 * std::sin(0.1))) < 1e-12);
  shifted.shared_slack = false;
  const auto separate = planning::buildPathOptimizerQp(
      references, two_circles, false, shifted);
  check(separate.slack_variables == 6);
  check(separate.constraints(6, 8) == 1.0);
  check(separate.constraints(15, 11) == 1.0);

  bool invalid_rejected = false;
  references[0].beta.clear();
  try {
    planning::buildPathOptimizerQp(references, {{0.0, 1.0}}, false, p);
  } catch (const std::invalid_argument &) {
    invalid_rejected = true;
  }
  check(invalid_rejected);
}
