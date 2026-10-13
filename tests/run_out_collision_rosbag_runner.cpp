// SPDX-License-Identifier: Apache-2.0
// K-City 경로와 예측 객체를 돌발 진입 충돌 코어에 연속 재생합니다.

#include "planning/run_out_collision.hpp"
#include "planning/run_out_state.hpp"
#include "planning/lanelet_bin_adapter.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iterator>
#include <stdexcept>
#include <string>

int main(int argc, char **argv) {
  if (argc < 3 || argc > 8) return 2;
  bool prepare = false;
  bool smooth = false;
  bool constant_ego_speed = false;
  bool analytical_forward = false;
  bool analytical_full = false;
  for (int index = 4; index < argc; ++index) {
    const std::string option(argv[index]);
    if (option == "--prepare-path") prepare = true;
    else if (option == "--smooth-path") smooth = true;
    else if (option == "--constant-ego-speed") constant_ego_speed = true;
    else if (option == "--analytical-forward") analytical_forward = true;
    else if (option == "--analytical-full") analytical_full = true;
    else return 2;
  }
  if (smooth && (analytical_forward || analytical_full)) return 2;
  if (analytical_forward && analytical_full) return 2;
  std::ifstream input(argv[1]);
  std::ofstream output(argv[2]);
  if (!input || !output) return 2;
  std::size_t frames = 0;
  input >> frames;
  planning::RunOutObjectFilterCore filter;
  planning::RunOutTemporalCore temporal;
  planning::RunOutCollisionParameters parameters;
  if (prepare) parameters.detection_distance_m = 1.0e9;
  planning::RunOutObjectFilterParameters filter_parameters;
  if (argc >= 4) {
    std::ifstream map_file(argv[3], std::ios::binary);
    if (!map_file) return 2;
    const std::vector<std::uint8_t> bytes(
        std::istreambuf_iterator<char>{map_file},
        std::istreambuf_iterator<char>{});
    const planning::LaneletBinAdapter map(bytes);
    for (const auto &segment : map.partitionSegments())
      filter_parameters.partition_segments.push_back({
          segment.from.x, segment.from.y,
          segment.to.x, segment.to.y});
    for (const auto id : map.crosswalkLaneletIds()) {
      const auto bounds = map.crosswalkBounds(id);
      std::vector<planning::CruisePredictedPose> polygon;
      for (const auto &point : bounds.left)
        polygon.push_back({point.x, point.y, 0.0});
      for (auto it = bounds.right.rbegin(); it != bounds.right.rend(); ++it)
        polygon.push_back({it->x, it->y, 0.0});
      if (polygon.size() >= 3)
        parameters.suppressed_polygons.push_back(std::move(polygon));
    }
  }
  for (std::size_t frame = 0; frame < frames; ++frame) {
    std::size_t path_count = 0, object_count = 0;
    double now_s = 0.0, ego_x = 0.0, ego_y = 0.0, ego_yaw = 0.0;
    double ego_velocity = 0.0, ego_acceleration = 0.0;
    input >> path_count >> object_count >> now_s >> ego_x >> ego_y >> ego_yaw >>
        ego_velocity >> ego_acceleration;
    planning::Trajectory path(path_count);
    for (auto &point : path) {
      double yaw = 0.0;
      input >> point.x >> point.y >> yaw >> point.longitudinal_velocity_mps;
      point.orientation_z = std::sin(yaw * 0.5);
      point.orientation_w = std::cos(yaw * 0.5);
      if (constant_ego_speed)
        point.longitudinal_velocity_mps = static_cast<float>(
            std::max(ego_velocity, 0.0));
    }
    std::vector<planning::RunOutCollisionObject> objects(object_count);
    for (auto &object : objects) {
      std::size_t path_total = 0;
      input >> object.id >> object.classification >> object.shape_type >>
          object.pose.x >> object.pose.y >> object.pose.yaw_rad >>
          object.length_m >> object.width_m >>
          object.minimum_velocity_mps >> object.maximum_velocity_mps >>
          path_total;
      object.predicted_paths.resize(path_total);
      for (auto &prediction : object.predicted_paths) {
        std::size_t pose_count = 0;
        input >> prediction.confidence >> prediction.time_step_s >> pose_count;
        prediction.poses.resize(pose_count);
        for (auto &pose : prediction.poses)
          input >> pose.x >> pose.y >> pose.yaw_rad;
      }
    }
    if (!input) throw std::runtime_error("돌발 진입 rosbag 입력이 유효하지 않습니다");
    auto collision_path = prepare ?
        planning::prepareRunOutCollisionPath(path, ego_x, ego_y) : path;
    if (smooth) {
      const auto smoothed = planning::smoothRunOutCollisionPath(
          collision_path, {ego_x, ego_y, ego_yaw},
          ego_velocity, ego_acceleration);
      if (smoothed) collision_path = *smoothed;
    } else if (analytical_forward || analytical_full) {
      collision_path = planning::smoothRunOutCollisionPathAnalytical(
          collision_path, {ego_x, ego_y, ego_yaw},
          ego_velocity, ego_acceleration, std::nullopt,
          analytical_full);
    }
    const auto filtered = filter.filter(collision_path,
        {ego_x, ego_y, ego_yaw}, now_s, objects,
        filter_parameters);
    const auto collision = planning::findRunOutCollision(
        collision_path, filtered, parameters);
    const auto unfiltered_collision = planning::findRunOutCollision(
        collision_path, objects, parameters);
    const bool accepted = temporal.acceptCollision(
        now_s, collision.has_value());
    if (collision)
      output << frame << ' ' << collision->id << ' ' <<
          collision->collision_arc_m << ' ' << collision->travel_time_s << ' ' <<
          collision->distance_from_vehicle_front_m << ' ' <<
          filtered.size() << ' ' << accepted << ' ' <<
          (unfiltered_collision ? unfiltered_collision->id : "-") << '\n';
    else
      output << frame << " - -1 -1 -1 " << filtered.size() <<
          " 0 " <<
          (unfiltered_collision ? unfiltered_collision->id : "-") << '\n';
  }
}
