// SPDX-License-Identifier: Apache-2.0
// 기록에서 추출한 경로·경계를 연속 입력하는 독립 실행기입니다.

#include "planning/path_optimizer_core.hpp"

#include <fstream>
#include <iomanip>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>

namespace {
planning::TrajectoryPoint readPoint(std::istream &stream) {
  planning::TrajectoryPoint point;
  stream >> point.x >> point.y >> point.z
         >> point.orientation_x >> point.orientation_y
         >> point.orientation_z >> point.orientation_w
         >> point.longitudinal_velocity_mps;
  if (!stream) throw std::runtime_error("경로 최적화 재생 점 입력 실패");
  return point;
}
void writeTrajectory(std::ostream &stream, const planning::Trajectory &points) {
  stream << points.size() << '\n';
  for (const auto &point : points)
    stream << point.x << ' ' << point.y << ' ' << point.z << ' '
           << point.orientation_z << ' ' << point.orientation_w << ' '
           << point.longitudinal_velocity_mps << '\n';
}
}  // 네임스페이스

int main(int argc, char **argv) {
  if (argc != 3 && argc != 4) return 2;
  const bool reset_policy = argc == 4 && std::string(argv[3]) == "--reset-policy";
  if (argc == 4 && !reset_policy) return 2;
  std::ifstream input(argv[1]);
  std::ofstream output(argv[2]);
  if (!input || !output) return 2;
  output << std::setprecision(17);
  planning::PathOptimizerCoreParameters parameters;
  parameters.vehicle_length = 4.77;
  parameters.rear_overhang = 1.03;
  parameters.left_overhang = 0.1;
  parameters.right_overhang = 0.1;
  parameters.qp.vehicle_width = 1.83;
  parameters.qp.wheelbase = 2.74;
  parameters.qp.optimization_center_offset = 2.74 * 0.8;
  parameters.qp.maximum_steering = 0.70;
  parameters.avoidance.wheelbase = 2.74;
  planning::PathOptimizerCore core(parameters);
  std::optional<planning::PathOptimizerCoreInput> previous_input;
  std::size_t reset_count = 0;
  std::size_t frames;
  input >> frames;
  for (std::size_t frame = 0; frame < frames; ++frame) {
    std::size_t point_count, left_count, right_count;
    int goal;
    input >> point_count >> left_count >> right_count >> goal;
    planning::PathOptimizerCoreInput data;
    data.goal_contained = goal != 0;
    data.ego = readPoint(input);
    input >> data.ego_speed;
    for (std::size_t i = 0; i < point_count; ++i)
      data.path.push_back(readPoint(input));
    for (std::size_t i = 0; i < left_count; ++i) {
      planning::PathOptimizerPoint2D point;
      input >> point.x >> point.y;
      data.left_bound.push_back(point);
    }
    for (std::size_t i = 0; i < right_count; ++i) {
      planning::PathOptimizerPoint2D point;
      input >> point.x >> point.y;
      data.right_bound.push_back(point);
    }
    if (!input) throw std::runtime_error("경로 최적화 재생 프레임 입력 실패");
    if (reset_policy &&
        planning::requiresPathOptimizerReset(data, previous_input)) {
      core.reset();
      ++reset_count;
    }
    previous_input = data;
    const auto result = core.optimize(data);
    output << (result ? 1 : 0) << '\n';
    if (result) {
      writeTrajectory(output, result->geometry.points);
      writeTrajectory(output, result->optimized);
      writeTrajectory(output, result->output);
    }
  }
  if (reset_policy)
    std::cerr << "이전 최적화 상태 초기화 " << reset_count << "회\n";
}
