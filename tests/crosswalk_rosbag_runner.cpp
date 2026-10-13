// SPDX-License-Identifier: Apache-2.0
// 기록된 경로와 예측 객체를 횡단보도 지도 계획기에 연속 전달합니다.

#include "planning/behavior_velocity_map_planner.hpp"

#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <string>
#include <vector>

int main(int argc, char **argv) {
  if (argc != 5) return 2;
  std::ifstream map_stream(argv[1], std::ios::binary);
  std::ifstream input(argv[2]);
  std::ofstream output(argv[3]);
  if (!map_stream || !input || !output) return 2;
  const bool rtc_enabled = std::string(argv[4]) == "1";
  output << std::setprecision(17);
  const std::vector<std::uint8_t> map_data(
      std::istreambuf_iterator<char>{map_stream},
      std::istreambuf_iterator<char>{});
  planning::BehaviorVelocityMapPlanner planner(
      map_data, {planning::BehaviorVelocityModule::Crosswalk});
  std::size_t frames = 0;
  input >> frames;
  for (std::size_t frame = 0; frame < frames; ++frame) {
    planning::BehaviorVelocityMapInput request;
    request.crosswalk_enable_rtc = rtc_enabled;
    std::size_t count = 0;
    input >> request.now_s >> request.ego_x >> request.ego_y >>
        request.ego_velocity_mps >> request.ego_acceleration_mps2 >> count;
    request.trajectory.resize(count);
    request.lane_ids.resize(count);
    for (std::size_t index = 0; index < count; ++index) {
      auto &point = request.trajectory[index];
      double yaw = 0.0;
      std::size_t lane_count = 0;
      input >> point.x >> point.y >> point.z >> yaw >>
          point.longitudinal_velocity_mps >> lane_count;
      point.orientation_z = std::sin(yaw * 0.5);
      point.orientation_w = std::cos(yaw * 0.5);
      request.lane_ids[index].resize(lane_count);
      for (auto &id : request.lane_ids[index]) input >> id;
    }
    std::size_t object_count = 0;
    input >> object_count;
    request.crosswalk_objects.reserve(object_count);
    for (std::size_t index = 0; index < object_count; ++index) {
      planning::BehaviorCrosswalkObject object;
      int classification = 0;
      std::size_t path_count = 0;
      input >> object.id >> object.position.x >> object.position.y >>
          classification >> object.predicted.length_m >>
          object.predicted.width_m >> object.predicted.velocity_x_mps >>
          object.predicted.velocity_y_mps >> path_count;
      object.classification = static_cast<std::uint8_t>(classification);
      object.predicted.predicted_paths.resize(path_count);
      for (auto &path : object.predicted.predicted_paths) {
        std::size_t pose_count = 0;
        input >> pose_count;
        path.resize(pose_count);
        for (auto &pose : path) input >> pose.x >> pose.y >> pose.yaw_rad;
      }
      request.crosswalk_objects.push_back(std::move(object));
    }
    if (!input || count < 2) return 2;
    const auto result = planner.apply(request);
    const auto first_zero = [](const planning::Trajectory &path) {
      for (std::size_t index = 0; index < path.size(); ++index) {
        if (std::abs(path[index].longitudinal_velocity_mps) < 0.01F)
          return index;
      }
      return path.size();
    };
    const std::size_t input_stop = first_zero(request.trajectory);
    const std::size_t output_stop = first_zero(result.trajectory);
    const bool stop_added = output_stop < result.trajectory.size() &&
        (input_stop == request.trajectory.size() ||
         std::hypot(result.trajectory[output_stop].x -
                        request.trajectory[input_stop].x,
                    result.trajectory[output_stop].y -
                        request.trajectory[input_stop].y) > 0.1);
    output << frame << ' ' << result.crosswalk_lanelet_ids.size() << ' '
           << stop_added;
    if (stop_added) {
      output << ' ' << result.trajectory[output_stop].x << ' '
             << result.trajectory[output_stop].y;
    }
    output << '\n';
  }
  return 0;
}
