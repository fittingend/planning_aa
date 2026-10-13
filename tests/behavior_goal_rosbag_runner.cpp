// SPDX-License-Identifier: Apache-2.0
// 기록된 목표 기준 경로에 독립 고정 목표 종점 계산을 적용합니다.

#include "planning/behavior_goal_refinement.hpp"
#include "planning/lanelet_bin_adapter.hpp"

#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <vector>

int main(int argc, char **argv) {
  if (argc != 4) return 2;
  std::ifstream map_stream(argv[1], std::ios::binary);
  std::ifstream input(argv[2]);
  std::ofstream output(argv[3]);
  if (!map_stream || !input || !output) return 2;
  const std::vector<std::uint8_t> map_data(
      std::istreambuf_iterator<char>{map_stream},
      std::istreambuf_iterator<char>{});
  planning::LaneletBinAdapter map(map_data);
  output << std::setprecision(17);
  std::size_t frames = 0;
  planning::BehaviorGoalPose goal;
  std::int64_t goal_lane_id = 0;
  double radius = 0.0;
  input >> frames >> goal.x >> goal.y >> goal.yaw_rad >>
      goal_lane_id >> radius;
  std::size_t lane_count = 0;
  input >> lane_count;
  std::vector<std::int64_t> route_lane_ids(lane_count);
  for (auto &id : route_lane_ids) input >> id;
  for (std::size_t frame = 0; frame < frames; ++frame) {
    std::size_t point_count = 0;
    input >> point_count;
    planning::BehaviorGoalPath path;
    path.points.resize(point_count);
    path.lane_ids.assign(point_count, route_lane_ids);
    for (auto &point : path.points) {
      input >> point.x >> point.y >> point.z >>
          point.orientation_x >> point.orientation_y >>
          point.orientation_z >> point.orientation_w >>
          point.longitudinal_velocity_mps >>
          point.lateral_velocity_mps >> point.heading_rate_rps;
    }
    if (!input) return 2;
    const auto refined = map.refineFixedGoalPath(
        path, goal, goal_lane_id, radius);
    output << refined.points.size() << '\n';
    for (const auto &point : refined.points) {
      output << point.x << ' ' << point.y << ' ' << point.z << ' '
             << point.longitudinal_velocity_mps << '\n';
    }
  }
  return 0;
}
