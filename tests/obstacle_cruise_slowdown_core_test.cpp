// SPDX-License-Identifier: Apache-2.0
// 횡거리별 속도와 장애물 감속 구간을 검사합니다.

#include "planning/obstacle_cruise_slowdown_core.hpp"

#include <cmath>
#include <stdexcept>

namespace {
void check(bool condition) {
  if (!condition) throw std::runtime_error("장애물 감속 계획 검사 실패");
}
}  // 익명 네임스페이스

int main() {
  planning::Trajectory path(101);
  for (std::size_t i = 0; i < path.size(); ++i) {
    path[i].x = static_cast<double>(i);
    path[i].longitudinal_velocity_mps = 10.0F;
  }
  planning::ObstacleCruiseSlowdownCandidate obstacle;
  obstacle.id = "정적차량";
  obstacle.front_collision_arc = 30.0;
  obstacle.back_collision_arc = 35.0;
  obstacle.lateral_distance = 0.2;
  planning::ObstacleCruiseSlowdownCore core;
  auto result = core.plan(path, {0.0, 0.0}, 10.0, 0.0, {obstacle});
  check(result.intervals.slowdowns.size() == 1);
  const auto &interval = result.intervals.slowdowns.front();
  check(std::abs(interval.from.x - 20.21) < 1e-9);
  check(std::abs(interval.to.x - 31.21) < 1e-9);
  check(interval.velocity >= 4.0 && interval.velocity < 10.0);
  check(result.trajectory.size() == 103);
  check(result.trajectory[25].longitudinal_velocity_mps < 10.0F);
  check(result.trajectory.back().longitudinal_velocity_mps == 10.0F);

  obstacle.longitudinal_velocity = 1.0;
  obstacle.lateral_distance = 0.2;
  core.reset();
  result = core.plan(path, {0.0, 0.0}, 10.0, 0.0, {obstacle});
  check(result.intervals.slowdowns.size() == 1);
  check(result.intervals.slowdowns.front().velocity >= 2.0);
  result = core.plan(path, {0.0, 0.0}, 10.0, 0.0, {});
  check(result.intervals.slowdowns.empty() && result.trajectory.size() == path.size());

  core.reset();
  obstacle.longitudinal_velocity = 0.0;
  obstacle.front_collision_arc = 5.0;
  obstacle.back_collision_arc = 10.0;
  result = core.plan(path, {0.0, 0.0}, 10.0, 0.0, {obstacle});
  check(result.intervals.slowdowns.size() == 1 &&
        result.intervals.slowdowns.front().from.x == 0.0);
  result = core.plan(path, {0.0, 0.0}, 10.0, 0.0, {obstacle});
  check(result.intervals.slowdowns.size() == 1 &&
        result.intervals.slowdowns.front().from.x == 0.0);
}
