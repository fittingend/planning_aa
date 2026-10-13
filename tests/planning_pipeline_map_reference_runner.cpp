// SPDX-License-Identifier: Apache-2.0
// 저장된 행동 속도 입력을 지도 플러그인과 후속 독립 단계에 전달합니다.

#include "planning/planning_pipeline_map_adapter.hpp"

#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <utility>
#include <vector>

int main(int argc, char **argv) {
  if (argc != 2) return 2;
  std::ifstream map_file(argv[1], std::ios::binary);
  if (!map_file) return 3;
  const std::vector<std::uint8_t> map_data{
      std::istreambuf_iterator<char>(map_file),
      std::istreambuf_iterator<char>()};
  planning::PlanningPipelineMapAdapter pipeline(
      map_data,
      {planning::BehaviorVelocityModule::StopSign,
       planning::BehaviorVelocityModule::TrafficLight});
  planning::PlanningPipelineFrame frame;
  planning::BehaviorVelocityMapInput context;
  std::size_t path_size = 0;
  if (!(std::cin >> frame.ego.x >> frame.ego.y >>
        frame.ego_speed_mps >> context.ego_acceleration_mps2 >>
        path_size)) return 4;
  frame.timestamp_s = 0.0;
  for (std::size_t index = 0; index < path_size; ++index) {
    planning::TrajectoryPoint point;
    std::size_t lane_count = 0;
    if (!(std::cin >> point.x >> point.y >> point.z >>
          point.orientation_x >> point.orientation_y >>
          point.orientation_z >> point.orientation_w >>
          point.longitudinal_velocity_mps >> point.lateral_velocity_mps >>
          point.heading_rate_rps >> lane_count)) return 5;
    std::vector<std::int64_t> ids(lane_count);
    for (auto &id : ids) {
      if (!(std::cin >> id)) return 6;
    }
    frame.behavior_path.points.push_back(point);
    frame.behavior_path.lane_ids.push_back(std::move(ids));
    const double yaw = std::atan2(
        2.0 * (point.orientation_w * point.orientation_z +
               point.orientation_x * point.orientation_y),
        1.0 - 2.0 * (point.orientation_y * point.orientation_y +
                     point.orientation_z * point.orientation_z));
    frame.left_bound.push_back({
        point.x - 6.0 * std::sin(yaw),
        point.y + 6.0 * std::cos(yaw)});
    frame.right_bound.push_back({
        point.x + 6.0 * std::sin(yaw),
        point.y - 6.0 * std::cos(yaw)});
  }
  if (frame.behavior_path.points.empty()) return 7;
  frame.ego.orientation_x = frame.behavior_path.points.front().orientation_x;
  frame.ego.orientation_y = frame.behavior_path.points.front().orientation_y;
  frame.ego.orientation_z = frame.behavior_path.points.front().orientation_z;
  frame.ego.orientation_w = frame.behavior_path.points.front().orientation_w;
  planning::PlanningPipelineMapResult result;
  for (int index = 0; index < 5; ++index) {
    frame.timestamp_s = index * 0.2;
    result = pipeline.onFrame(frame, context);
  }
  if (!result.behavior_velocity) return 8;
  std::cout << std::setprecision(17)
            << static_cast<int>(result.pipeline.status) << ' '
            << result.behavior_velocity->trajectory.size() << ' '
            << result.pipeline.elastic_band.size() << ' '
            << result.pipeline.optimized.size() << ' '
            << result.pipeline.motion_velocity.size() << ' '
            << result.pipeline.cruise_stopped.size() << ' '
            << (result.pipeline.validation ? 1 : 0) << '\n';
  for (const auto &point : result.behavior_velocity->trajectory) {
    std::cout << point.x << ' ' << point.y << ' ' << point.z << ' '
              << point.orientation_x << ' ' << point.orientation_y << ' '
              << point.orientation_z << ' ' << point.orientation_w << ' '
              << point.longitudinal_velocity_mps << ' '
              << point.lateral_velocity_mps << ' '
              << point.heading_rate_rps << '\n';
  }
}
