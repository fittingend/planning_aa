// SPDX-License-Identifier: Apache-2.0
// 기록된 평활화 궤적에서 경로 최적화 기준점 형상을 계산합니다.

#include "planning/path_optimizer_reference_geometry.hpp"

#include <fstream>
#include <iomanip>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
planning::Trajectory readPoints(const std::string &filename) {
  std::ifstream input(filename);
  if (!input) throw std::runtime_error("기준 경로 CSV를 열 수 없습니다");
  planning::Trajectory points;
  std::string line;
  while (std::getline(input, line)) {
    std::vector<double> values;
    std::size_t start = 0;
    while (start <= line.size()) {
      const auto end = line.find(',', start);
      values.push_back(std::stod(line.substr(start, end - start)));
      if (end == std::string::npos) break;
      start = end + 1;
    }
    if (values.size() != 8) throw std::runtime_error("기준 경로 CSV 열 수가 다릅니다");
    planning::TrajectoryPoint point;
    point.x = values[0]; point.y = values[1]; point.z = values[2];
    point.orientation_x = values[3]; point.orientation_y = values[4];
    point.orientation_z = values[5]; point.orientation_w = values[6];
    point.longitudinal_velocity_mps = static_cast<float>(values[7]);
    points.push_back(point);
  }
  return points;
}
}  // 네임스페이스

int main(int argc, char **argv) {
  if (argc != 5 && argc != 6) return 2;
  const auto points = readPoints(argv[1]);
  planning::TrajectoryPoint ego;
  ego.x = std::stod(argv[3]);
  ego.y = std::stod(argv[4]);
  planning::PathOptimizerReferenceGeometry previous;
  const planning::PathOptimizerReferenceGeometry *previous_ptr = nullptr;
  if (argc == 6) {
    previous.points = readPoints(argv[5]);
    previous.curvature.resize(previous.points.size(), 0.0);
    previous_ptr = &previous;
  }
  const auto result = planning::preparePathOptimizerReferenceGeometry(
      points, ego, {}, previous_ptr);
  std::ofstream output(argv[2]);
  if (!output) return 4;
  output << std::setprecision(17);
  for (std::size_t i = 0; i < result.points.size(); ++i) {
    const auto &point = result.points[i];
    output << point.x << ',' << point.y << ',' << point.z << ','
           << point.longitudinal_velocity_mps << ','
           << result.curvature[i] << ',' << result.arc_length_to_next[i] << ','
           << point.orientation_z << ',' << point.orientation_w << '\n';
  }
}
