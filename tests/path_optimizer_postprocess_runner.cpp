// SPDX-License-Identifier: Apache-2.0
// 기록된 경로 최적화 내부 궤적에 독립 후처리를 적용합니다.

#include "planning/path_optimizer_solution.hpp"

#include <fstream>
#include <iomanip>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
planning::Trajectory readTrajectory(const std::string &name) {
  std::ifstream stream(name);
  if (!stream) throw std::runtime_error("궤적 CSV를 열 수 없습니다");
  planning::Trajectory points;
  std::string line;
  while (std::getline(stream, line)) {
    std::size_t start = 0;
    std::vector<double> values;
    while (start <= line.size()) {
      const auto end = line.find(',', start);
      values.push_back(std::stod(line.substr(start, end - start)));
      if (end == std::string::npos) break;
      start = end + 1;
    }
    if (values.size() != 8) throw std::runtime_error("궤적 CSV 열 수가 다릅니다");
    planning::TrajectoryPoint point;
    point.x = values[0];
    point.y = values[1];
    point.z = values[2];
    point.orientation_x = values[3];
    point.orientation_y = values[4];
    point.orientation_z = values[5];
    point.orientation_w = values[6];
    point.longitudinal_velocity_mps = static_cast<float>(values[7]);
    points.push_back(point);
  }
  return points;
}
}  // 네임스페이스

int main(int argc, char **argv) {
  if (argc != 6) return 2;
  const auto input = readTrajectory(argv[1]);
  const auto optimized = readTrajectory(argv[2]);
  planning::TrajectoryPoint ego;
  ego.x = std::stod(argv[4]);
  ego.y = std::stod(argv[5]);
  const auto output = planning::finalizePathOptimizerTrajectory(
      input, optimized, ego);
  std::ofstream stream(argv[3]);
  if (!stream) return 3;
  stream << std::setprecision(17);
  for (const auto &point : output) {
    stream << point.x << ',' << point.y << ',' << point.z << ','
           << point.longitudinal_velocity_mps << '\n';
  }
}
