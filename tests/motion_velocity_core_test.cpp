// SPDX-License-Identifier: Apache-2.0
// 정지점과 감속 구간을 차례로 삽입할 때 속도가 증가하지 않는지 검사합니다.

#include "planning/motion_velocity_core.hpp"

#include <cmath>
#include <stdexcept>

namespace {
void check(bool condition) {
  if (!condition) throw std::runtime_error("동작 속도 계획 병합 검사 실패");
}
}  // 네임스페이스

int main() {
  planning::Trajectory input(5);
  for (std::size_t i = 0; i < input.size(); ++i) {
    input[i].x = 2.0 * i;
    input[i].longitudinal_velocity_mps = 8.0F;
  }
  planning::MotionVelocityPlan plan;
  plan.slowdowns.push_back({{1.0, 0.0}, {5.0, 0.0}, 3.0});
  plan.stop_points.push_back({7.0, 0.0});
  const auto output = planning::applyMotionVelocityPlans(input, {plan});
  check(output.size() == 8);
  check(std::abs(output[1].x - 1.0) < 1e-12);
  check(std::abs(output[1].longitudinal_velocity_mps - 3.0F) < 1e-12);
  check(std::abs(output[4].x - 5.0) < 1e-12);
  check(std::abs(output[4].longitudinal_velocity_mps - 3.0F) < 1e-12);
  check(std::abs(output[6].x - 7.0) < 1e-12);
  check(output[6].longitudinal_velocity_mps == 0.0F);
  check(output[7].longitudinal_velocity_mps == 0.0F);
  const auto passthrough = planning::applyMotionVelocityPlans(input, {});
  check(passthrough.size() == input.size());
  planning::Trajectory dense(5);
  for (std::size_t i = 0; i < dense.size(); ++i) dense[i].x = i * 0.25;
  check(planning::resampleMotionVelocityPlanningInput(dense).size() == 2);
}
