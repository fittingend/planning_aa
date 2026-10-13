// SPDX-License-Identifier: Apache-2.0
// RSS 거리와 순항 대상 선택·PID 속도 제한을 검사합니다.

#include "planning/obstacle_cruise_pid_core.hpp"

#include <cmath>
#include <stdexcept>

namespace {
void check(bool condition) {
  if (!condition) throw std::runtime_error("장애물 순항 PID 검사 실패");
}
}  // 익명 네임스페이스

int main() {
  planning::ObstacleCruisePidCore core;
  const auto first = core.update(10.0, {{50.0, 8.0}, {40.0, 8.0}});
  check(first && std::abs(first->obstacle_distance - 40.0) < 1e-12);
  check(std::abs(first->target_distance - 42.0) < 1e-12);
  check(std::abs(first->error_distance + 2.0) < 1e-12);
  check(std::abs(first->maximum_velocity - 9.975) < 1e-12);
  check(first->minimum_acceleration == -2.0);
  const auto second = core.update(10.0, {{40.0, 8.0}});
  check(second && std::abs(second->maximum_velocity - 9.975) < 1e-12);
  check(!core.update(10.0, {}));
  const auto restored = core.update(10.0, {{40.0, 8.0}});
  check(restored && std::abs(restored->maximum_velocity - 9.975) < 1e-12);
  core.reset();
  const auto accelerated = core.update(10.0, {{50.0, 8.0}});
  check(accelerated && accelerated->maximum_velocity > 10.0 &&
        accelerated->maximum_velocity < 10.2);
  planning::CruisePidParameters recorded_parameters;
  recorded_parameters.safe_distance_margin = 6.0;
  planning::ObstacleCruisePidCore recorded_core(recorded_parameters);
  const auto recorded = recorded_core.update(10.0, {{40.0, 8.0}});
  check(recorded && std::abs(recorded->target_distance - 44.0) < 1e-12);
}
