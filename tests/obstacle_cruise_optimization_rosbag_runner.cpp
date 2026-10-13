// SPDX-License-Identifier: Apache-2.0
// 기록된 순항 거리와 경로 길이로 선택적 최적화 QP를 계산합니다.

#include "planning/obstacle_cruise_optimization.hpp"

#include <fstream>
#include <stdexcept>

int main(int argc, char **argv) {
  if (argc != 3) return 2;
  std::ifstream input(argv[1]);
  std::ofstream output(argv[2]);
  if (!input || !output) return 2;
  std::size_t count = 0;
  input >> count;
  const auto time = planning::cruiseOptimizationTimeVector();
  for (std::size_t i = 0; i < count; ++i) {
    double ego_speed = 0.0, object_distance = 0.0;
    double object_speed = 0.0, trajectory_length = 0.0;
    double maximum_velocity = 0.0;
    input >> ego_speed >> object_distance >> object_speed >>
        trajectory_length >> maximum_velocity;
    if (!input)
      throw std::runtime_error("순항 최적화 기록 입력이 중간에서 끝났습니다");
    planning::CruiseOptimizationObstacle object;
    object.velocity_mps = object_speed;
    object.collision_points.push_back({object_distance, 0.0, 0.0});
    planning::CruiseOptimizationInput problem;
    problem.time_s = time;
    problem.boundaries = planning::cruiseOptimizationBoundaries(
        time, trajectory_length, {object});
    problem.initial_velocity_mps = ego_speed;
    problem.maximum_velocity_mps = maximum_velocity;
    const auto optimized = planning::optimizeCruiseVelocity(problem);
    output << (optimized ? 1 : 0) << ' '
           << (optimized ? optimized->distance_m.back() : -1.0) << ' '
           << (optimized ? optimized->velocity_mps.front() : -1.0) << '\n';
  }
}
