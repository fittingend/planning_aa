// SPDX-License-Identifier: Apache-2.0
// 원본 순항 디버그 배열의 원시 값으로 PID 상태를 연속 재생합니다.

#include "planning/obstacle_cruise_pid_core.hpp"

#include <fstream>
#include <iomanip>
#include <stdexcept>

int main(int argc, char **argv) {
  if (argc != 3) return 2;
  std::ifstream input(argv[1]);
  std::ofstream output(argv[2]);
  if (!input || !output) return 2;
  output << std::setprecision(17);
  planning::CruisePidParameters parameters;
  parameters.safe_distance_margin = 6.0;
  planning::ObstacleCruisePidCore core(parameters);
  std::size_t count = 0;
  input >> count;
  for (std::size_t i = 0; i < count; ++i) {
    int active = 0;
    double ego_velocity = 0.0;
    double distance = 0.0;
    double obstacle_velocity = 0.0;
    input >> active >> ego_velocity >> distance >> obstacle_velocity;
    if (!input) throw std::runtime_error("순항 PID 기록 입력이 중간에서 끝났습니다");
    const auto result = active
        ? core.update(ego_velocity, {{distance, obstacle_velocity}})
        : core.update(ego_velocity, {});
    output << (result ? 1 : 0) << ' '
           << (result ? result->maximum_velocity : 0.0) << ' '
           << (result ? result->minimum_acceleration : 0.0) << ' '
           << (result ? result->target_distance : 0.0) << ' '
           << (result ? result->obstacle_distance : 0.0) << ' '
           << (result ? result->error_distance : 0.0) << '\n';
  }
}
