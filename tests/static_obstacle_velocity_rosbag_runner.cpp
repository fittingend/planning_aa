// SPDX-License-Identifier: Apache-2.0
// K-City 경로의 지도 정적 장애물 감속 요청 수를 계산합니다.

#include "planning/lanelet_bin_adapter.hpp"

#include <cstdint>
#include <fstream>
#include <iostream>
#include <iterator>
#include <stdexcept>
#include <string>
#include <vector>

int main(int argc, char **argv) {
  if (argc != 4 && argc != 5) return 2;
  const bool bicycle = argc == 5 && std::string(argv[4]) == "--bicycle";
  if (argc == 5 && !bicycle) return 2;
  std::ifstream map_stream(argv[1], std::ios::binary);
  std::ifstream input(argv[2]);
  std::ofstream output(argv[3]);
  if (!map_stream || !input || !output) return 2;
  const std::vector<std::uint8_t> map_data(
      std::istreambuf_iterator<char>(map_stream), {});
  planning::LaneletBinAdapter map(map_data);
  planning::ObstacleVelocityLimiterParameters parameters;
  if (bicycle)
    parameters.model = planning::VelocityLimiterProjectionModel::Bicycle;
  std::size_t frame_count = 0;
  input >> frame_count;
  for (std::size_t frame = 0; frame < frame_count; ++frame) {
    std::size_t count = 0;
    std::size_t ego_index = 0;
    double ego_velocity = 0.0;
    input >> count >> ego_index >> ego_velocity;
    planning::Trajectory path(count);
    std::vector<double> times(count);
    for (std::size_t i = 0; i < count; ++i)
      input >> path[i].x >> path[i].y >> path[i].orientation_z >>
          path[i].orientation_w >> path[i].longitudinal_velocity_mps >>
          times[i] >> path[i].front_wheel_angle_rad;
    if (!input) throw std::runtime_error("정적 장애물 재생 입력이 중간에서 끝났습니다");
    const auto plan = map.planStaticObstacleVelocityLimit(
        path, times, ego_index, ego_velocity, parameters);
    output << plan.slowdowns.size() << '\n';
  }
}
