// SPDX-License-Identifier: Apache-2.0
// 원본 경로 최적화의 상태 방정식과 동일한 계수를 검사합니다.

#include "planning/path_optimizer_model.hpp"

#include <cmath>
#include <stdexcept>
#include <vector>

namespace {
void check(const bool condition) {
  static int check_number = 0;
  ++check_number;
  if (!condition) {
    throw std::runtime_error(
        "경로 최적화 상태 방정식 검사에 실패했습니다: " +
        std::to_string(check_number));
  }
}
}  // 네임스페이스

int main() {
  const double wheelbase = 2.79;
  const auto step = planning::bicycleStateStep(wheelbase, 0.6, 0.0, 1.5);
  check(step.transition(0, 1) == 1.5);
  check(std::abs(step.steering(1) - 1.5 / wheelbase) < 1e-12);
  check(step.offset.norm() == 0.0);

  const double curvature = 0.1;
  const double ds = 2.0;
  const auto curved = planning::bicycleStateStep(wheelbase, 0.2, curvature, ds);
  const double delta = std::atan(wheelbase * curvature);
  const double limited = 0.2;
  const double expected_steering = ds / wheelbase / (std::cos(delta) * std::cos(delta));
  const double expected_offset = -ds * curvature + ds / wheelbase *
      (std::tan(limited) - limited / (std::cos(limited) * std::cos(limited)));
  check(std::abs(curved.steering(1) - expected_steering) < 1e-12);
  check(std::abs(curved.offset(1) - expected_offset) < 1e-12);

  const auto equation = planning::buildPathOptimizerStateEquation(
      std::vector<double>{1.0, 2.0}, wheelbase, 0.6);
  check(equation.transition.rows() == 6);
  check(equation.steering.cols() == 2);
  check(equation.transition.block<2, 2>(0, 0).isIdentity());
  check(equation.transition(2, 1) == 1.0);
  check(equation.transition(4, 3) == 2.0);
  check(std::abs(equation.steering(3, 0) - 1.0 / wheelbase) < 1e-12);
  check(std::abs(equation.steering(5, 1) - 2.0 / wheelbase) < 1e-12);
  check(equation.offset.norm() == 0.0);
  bool rejected = false;
  try {
    planning::buildPathOptimizerStateEquation({-1.0}, wheelbase, 0.6);
  } catch (const std::invalid_argument &) {
    rejected = true;
  }
  check(rejected);
}
