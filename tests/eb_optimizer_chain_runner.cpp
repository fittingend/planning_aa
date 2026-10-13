// SPDX-License-Identifier: Apache-2.0
// 기록된 행동 속도 경로를 두 독립 계산 단계에 연속 전달합니다.

#include "planning/elastic_band_core.hpp"
#include "planning/path_optimizer_core.hpp"

#include <fstream>
#include <iomanip>
#include <optional>
#include <stdexcept>
#include <string>

namespace {
planning::TrajectoryPoint readPoint(std::istream &input) {
  planning::TrajectoryPoint point;
  input >> point.x >> point.y >> point.z
        >> point.orientation_x >> point.orientation_y
        >> point.orientation_z >> point.orientation_w
        >> point.longitudinal_velocity_mps;
  if (!input) throw std::runtime_error("연속 경로 점 입력 실패");
  return point;
}
}

int main(int argc, char **argv) {
  if (argc != 3 && argc != 4) return 2;
  const bool recorded_optimizer_input = argc == 4 &&
      std::string(argv[3]) == "--optimizer-recorded-smooth";
  if (argc == 4 && !recorded_optimizer_input) return 2;
  std::ifstream input(argv[1]);
  std::ofstream output(argv[2]);
  if (!input || !output) return 2;
  output << std::setprecision(17);
  planning::ElasticBandPlanner elastic;
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
  planning::PathOptimizerCore optimizer(parameters);
  std::optional<planning::PathOptimizerCoreInput> previous_optimizer_input;
  bool saw_recorded_smooth = false;
  std::size_t frames = 0;
  input >> frames;
  for (std::size_t frame = 0; frame < frames; ++frame) {
    double time_s = 0.0;
    int goal = 0;
    std::size_t path_count = 0, left_count = 0, right_count = 0;
    input >> time_s >> goal >> path_count >> left_count >> right_count;
    auto ego = readPoint(input);
    auto optimizer_ego = readPoint(input);
    const double ego_speed = ego.longitudinal_velocity_mps;
    planning::Trajectory path;
    for (std::size_t index = 0; index < path_count; ++index)
      path.push_back(readPoint(input));
    planning::PathOptimizerCoreInput optimizer_input;
    for (std::size_t index = 0; index < left_count; ++index) {
      planning::PathOptimizerPoint2D point;
      input >> point.x >> point.y;
      optimizer_input.left_bound.push_back(point);
    }
    for (std::size_t index = 0; index < right_count; ++index) {
      planning::PathOptimizerPoint2D point;
      input >> point.x >> point.y;
      optimizer_input.right_bound.push_back(point);
    }
    planning::Trajectory recorded_smooth;
    if (recorded_optimizer_input) {
      std::size_t recorded_count = 0;
      input >> recorded_count;
      for (std::size_t index = 0; index < recorded_count; ++index)
        recorded_smooth.push_back(readPoint(input));
    }
    if (!input) throw std::runtime_error("연속 경로 경계 입력 실패");
    const auto smoothed = elastic.run(path, ego, ego_speed, time_s);
    if (recorded_optimizer_input && !recorded_smooth.empty() &&
        !saw_recorded_smooth) {
      optimizer.reset();
      previous_optimizer_input.reset();
      saw_recorded_smooth = true;
    }
    optimizer_input.path = recorded_optimizer_input &&
        !recorded_smooth.empty() ? recorded_smooth : smoothed.trajectory;
    optimizer_input.ego = optimizer_ego;
    optimizer_input.ego_speed = optimizer_ego.longitudinal_velocity_mps;
    optimizer_input.goal_contained = goal != 0;
    if (planning::requiresPathOptimizerReset(
            optimizer_input, previous_optimizer_input))
      optimizer.reset();
    previous_optimizer_input = optimizer_input;
    const auto result = optimizer.optimize(optimizer_input);
    output << smoothed.trajectory.size() << ' ' << (result ? 1 : 0)
           << '\n';
    for (const auto &point : smoothed.trajectory) {
      output << point.x << ' ' << point.y << ' '
             << point.longitudinal_velocity_mps << '\n';
    }
    if (result) {
      output << result->output.size() << '\n';
      for (const auto &point : result->output) {
        output << point.x << ' ' << point.y << ' '
               << point.longitudinal_velocity_mps << '\n';
      }
    }
  }
}
