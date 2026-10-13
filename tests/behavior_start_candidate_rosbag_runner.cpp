// SPDX-License-Identifier: Apache-2.0
// 기록된 인식 객체와 지도에서 출발 후보 수를 독립 계산합니다.

#include "planning/behavior_path_lane_safety.hpp"
#include "planning/behavior_candidate_callback.hpp"
#include "planning/lanelet_bin_adapter.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iterator>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

int main(int argc, char **argv) {
  if (argc != 4 && argc != 5) return 2;
  std::ifstream map_stream(argv[1], std::ios::binary);
  std::ifstream input(argv[2]);
  std::ofstream output(argv[3]);
  if (!map_stream || !input || !output) return 2;
  const std::vector<std::uint8_t> map_data(
      std::istreambuf_iterator<char>{map_stream},
      std::istreambuf_iterator<char>{});
  const planning::LaneletBinAdapter map(map_data);
  double start_x = 0.0;
  double start_y = 0.0;
  std::int64_t lane_id = 0;
  std::size_t frame_count = 0;
  input >> start_x >> start_y >> lane_id >> frame_count;
  if (!input) return 2;
  std::string header_tail;
  std::getline(input, header_tail);
  planning::BehaviorGoalPose goal;
  const bool shared_callback = static_cast<bool>(
      std::istringstream(header_tail) >> goal.x >> goal.y >> goal.yaw_rad);
  planning::StartPullOutCandidatePlanInput plan;
  if (argc == 5) plan.maximum_backward_distance_m = std::stod(argv[4]);
  const auto centerline = map.laneCenterline(lane_id);
  for (const auto &point : centerline) {
    plan.centerline.push_back({point.x, point.y});
  }
  plan.lane_polygons = map.lanePolygons({lane_id});
  plan.original_start_pose = {0.0, start_x, start_y, 0.0, 0.0};
  std::vector<double> arcs(centerline.size(), 0.0);
  for (std::size_t index = 1; index < centerline.size(); ++index) {
    arcs[index] = arcs[index - 1] + std::hypot(
        centerline[index].x - centerline[index - 1].x,
        centerline[index].y - centerline[index - 1].y);
  }
  plan.lane_length_m = arcs.back();
  double best_distance = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0; index + 1 < centerline.size(); ++index) {
    const double dx = centerline[index + 1].x - centerline[index].x;
    const double dy = centerline[index + 1].y - centerline[index].y;
    const double length = arcs[index + 1] - arcs[index];
    const double fraction = std::clamp(
        ((start_x - centerline[index].x) * dx +
         (start_y - centerline[index].y) * dy) / (length * length),
        0.0, 1.0);
    const double distance = std::hypot(
        start_x - centerline[index].x - fraction * dx,
        start_y - centerline[index].y - fraction * dy);
    if (distance < best_distance) {
      best_distance = distance;
      plan.start_lane_arc_m = arcs[index] + fraction * length;
    }
  }
  for (std::size_t frame = 0; frame < frame_count; ++frame) {
    std::size_t count = 0;
    input >> count;
    std::string frame_tail;
    std::getline(input, frame_tail);
    std::int64_t callback_time_ns = 0;
    std::int64_t perception_time_ns = 0;
    if (shared_callback &&
        !(std::istringstream(frame_tail) >> callback_time_ns >>
          perception_time_ns)) return 2;
    plan.objects.resize(count);
    for (auto &object : plan.objects) {
      std::size_t footprint_count = 0;
      input >> object.pose.x >> object.pose.y >> object.pose.yaw_rad >>
          object.velocity_x_mps >> object.velocity_y_mps >>
          object.shape.shape_type >> object.shape.length_m >>
          object.shape.width_m >> object.allowed_class >> footprint_count;
      object.shape.footprint.resize(footprint_count);
      for (auto &[x, y] : object.shape.footprint) input >> x >> y;
    }
    if (!input) return 2;
    const auto selected_objects = planning::filterStartPullOutStopObjects(
        plan.objects, plan.lane_polygons, plan.centerline,
        plan.original_start_pose.x, plan.original_start_pose.y,
        plan.moving_object_threshold_mps,
        plan.backward_path_length_m + plan.maximum_backward_distance_m,
        std::numeric_limits<double>::max());
    if (shared_callback) {
      planning::BehaviorCandidateCallbackInput callback;
      callback.callback_time_ns = callback_time_ns;
      callback.maximum_perception_age_ns = 1'000'000'000;
      callback.perception = {perception_time_ns, plan.objects};
      callback.start = plan;
      callback.goal.goal = goal;
      callback.goal.object_hard_margin_m = 0.1;
      const auto result = planning::evaluateBehaviorCandidateCallback(
          callback);
      output << result.start_candidates.size() << ' '
             << selected_objects.size() << ' '
             << static_cast<int>(result.goal_safety) << '\n';
    } else {
      const auto candidates = planning::planStartPullOutPoseCandidates(plan);
      output << candidates.size() << ' ' << selected_objects.size() << '\n';
    }
  }
  return 0;
}
