// SPDX-License-Identifier: Apache-2.0
// 기록에서 역산한 정지 거리로 정지점 삽입을 재실행합니다.

#include "planning/obstacle_cruise_stop_core.hpp"

#include <fstream>
#include <iomanip>
#include <stdexcept>

int main(int argc, char **argv) {
  if (argc != 3) return 2;
  std::ifstream input(argv[1]);
  std::ofstream output(argv[2]);
  if (!input || !output) return 2;
  output << std::setprecision(17);
  std::size_t frame_count;
  input >> frame_count;
  for (std::size_t frame = 0; frame < frame_count; ++frame) {
    std::size_t count;
    double stop_distance;
    input >> count >> stop_distance;
    planning::Trajectory path(count);
    for (auto &point : path)
      input >> point.x >> point.y >> point.z
            >> point.orientation_x >> point.orientation_y
            >> point.orientation_z >> point.orientation_w
            >> point.longitudinal_velocity_mps;
    if (!input) throw std::runtime_error("장애물 정지 재생 입력 실패");
    planning::ObstacleCruiseStopCore core;
    const auto result = core.plan(
        path, 0.0, 2.0, {{0, stop_distance, 0.0}});
    output << result.trajectory.size() << '\n';
    for (const auto &point : result.trajectory)
      output << point.x << ' ' << point.y << ' '
             << point.longitudinal_velocity_mps << '\n';
  }
}
