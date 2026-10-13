// SPDX-License-Identifier: Apache-2.0
// 직선 경로를 두 주기 연속 풀어 이전 고정 상태까지 확인합니다.

#include "planning/path_optimizer_core.hpp"

#include <cmath>
#include <stdexcept>

namespace {
void check(bool condition) {
  if (!condition) throw std::runtime_error("경로 최적화 연결 검사 실패");
}
}  // 네임스페이스

int main() {
  planning::PathOptimizerCoreParameters parameters;
  check(std::abs(parameters.vehicle_length - 4.89) < 1e-12 &&
        std::abs(parameters.rear_overhang - 1.10) < 1e-12 &&
        std::abs(parameters.qp.vehicle_width - 1.896) < 1e-12 &&
        std::abs(parameters.qp.maximum_steering - 0.70) < 1e-12);
  parameters.geometry.forward_length = 20.0;
  parameters.geometry.maximum_points = 20;
  parameters.minimum_drivable_width = 1.0;
  parameters.maximum_bound_fixing_time = 1.0;
  planning::PathOptimizerCore core(parameters);
  planning::PathOptimizerCoreInput input;
  for (int i = 0; i <= 100; ++i) {
    planning::TrajectoryPoint point;
    point.x = static_cast<double>(i) * 0.5;
    point.longitudinal_velocity_mps = 5.0F;
    input.path.push_back(point);
  }
  input.left_bound = {{-10.0, 3.0}, {60.0, 3.0}};
  input.right_bound = {{-10.0, -3.0}, {60.0, -3.0}};
  input.ego.x = 10.0;
  input.ego_speed = 3.0;
  check(planning::requiresPathOptimizerReset(input, std::nullopt));
  check(!planning::requiresPathOptimizerReset(input, input));
  auto shifted = input;
  for (auto &point : shifted.path) point.y += 2.1;
  check(planning::requiresPathOptimizerReset(shifted, input));
  shifted = input;
  shifted.ego.x += 5.1;
  check(planning::requiresPathOptimizerReset(shifted, input));
  shifted = input;
  shifted.ego_speed = 0.0;
  shifted.path.back().x += 15.0;
  check(planning::requiresPathOptimizerReset(shifted, input));
  const auto first = core.optimize(input);
  check(first.has_value());
  check(first->optimized.size() == first->geometry.points.size());
  check(first->geometry.arc_length_to_next.back() > 0.5);
  check(!first->output.empty());
  check(first->geometry.optimized_states.size() == first->geometry.points.size());
  input.ego.x = 11.0;
  const auto second = core.optimize(input);
  check(second.has_value());
  check(second->geometry.fixed_states.front().has_value());
  check(second->solve.solved);
  for (const auto &point : second->optimized) {
    check(std::isfinite(point.x) && std::isfinite(point.y));
  }
}
