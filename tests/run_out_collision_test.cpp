// SPDX-License-Identifier: Apache-2.0
// 돌발 진입 예측 경로 보간과 차량 앞쪽 충돌을 확인합니다.

#include "planning/run_out_collision.hpp"
#include "planning/run_out_obstacle_creator.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

int main() {
  using namespace planning;
  const auto require = [](bool condition) {
    if (!condition) throw std::runtime_error("돌발 진입 충돌 검사 실패");
  };
  CruisePredictedPath slow;
  slow.confidence = 0.2;
  slow.poses = {{0.0, 0.0, 0.0}, {0.0, 10.0, 0.0}};
  CruisePredictedPath selected;
  selected.confidence = 0.9;
  selected.poses = {{0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}};
  const auto predicted = predictRunOutObjectPose({slow, selected}, 2.0, 1.0);
  require(predicted.has_value());
  require(std::abs(predicted->x - 2.0) < 1e-9);
  require(std::abs(predicted->y) < 1e-9);
  require(!predictRunOutObjectPose({}, 1.0, 1.0));
  const auto velocity = runOutVelocityRange(2.0, -1.0, 0.25, 0.0);
  require(std::abs(velocity.minimum_mps - std::hypot(1.02, 1.0)) < 1e-9);
  require(std::abs(velocity.maximum_mps - std::hypot(2.98, 1.0)) < 1e-9);

  Trajectory path(10);
  for (std::size_t i = 0; i < path.size(); ++i) {
    path[i].x = static_cast<double>(i);
    path[i].longitudinal_velocity_mps = 2.0F;
  }
  const auto prepared = prepareRunOutCollisionPath(path, 0.1, 0.0, 0.35, 4.0);
  require(prepared.size() == 6);
  require(std::abs(prepared.back().x - 5.0) < 1e-9);
  const auto smoothed = smoothRunOutCollisionPath(
      path, {0.0, 0.0, 0.0}, 2.0, 0.0, 1.5);
  require(smoothed.has_value() && smoothed->size() >= 2);
  require(std::abs(smoothed->front().x) < 1e-6);
  require(std::all_of(smoothed->begin(), smoothed->end(), [](const auto &point) {
    return std::isfinite(point.longitudinal_velocity_mps) &&
           point.longitudinal_velocity_mps <= 1.5F + 1e-5F;
  }));
  const auto analytical = smoothRunOutCollisionPathAnalytical(
      path, {0.0, 0.0, 0.0}, 1.0, 0.0, 1.5);
  require(analytical.size() >= path.size());
  require(std::all_of(analytical.begin(), analytical.end(), [](const auto &point) {
    return std::isfinite(point.longitudinal_velocity_mps) &&
           point.longitudinal_velocity_mps <= 1.5F + 1e-5F;
  }));
  RunOutCollisionObject object;
  object.id = "보행자";
  object.pose = {4.5, 0.6, 0.0};
  object.predicted_paths = {{{object.pose, object.pose}, 1.0, 1.0}};
  object.shape_type = 1;
  object.length_m = 0.8;
  object.width_m = 0.8;
  object.minimum_velocity_mps = 0.0;
  object.maximum_velocity_mps = 0.0;
  const auto collision = findRunOutCollision(path, {object});
  require(collision.has_value());
  require(collision->id == "보행자");
  require(collision->collision_arc_m > 4.0 &&
          collision->collision_arc_m < 5.0);
  require(std::abs(collision->travel_time_s - 0.5) < 1e-9);
  require(collision->distance_from_vehicle_front_m < 1.0);
  RunOutCollisionParameters crossing_parameters;
  crossing_parameters.suppressed_polygons = {{
      {3.0, -2.0, 0.0}, {6.0, -2.0, 0.0},
      {6.0, 2.0, 0.0}, {3.0, 2.0, 0.0}}};
  require(!findRunOutCollision(path, {object}, crossing_parameters));
  RunOutObjectFilterCore filter;
  auto on_path = object;
  on_path.pose = {4.5, 0.2, 0.0};
  on_path.predicted_paths.front().poses = {on_path.pose, on_path.pose};
  require(filter.filter(path, {0.0, 0.0, 0.0}, 0.0,
                        {on_path}).size() == 1);
  require(filter.filter(path, {0.0, 0.0, 0.0}, 1.1,
                        {on_path}).empty());
  on_path.pose.y = 2.0;
  require(filter.filter(path, {0.0, 0.0, 0.0}, 1.2,
                        {on_path}).size() == 1);
  RunOutObjectFilterParameters partition_parameters;
  partition_parameters.partition_segments.push_back({0.0, 1.0, 9.0, 1.0});
  require(filter.filter(path, {0.0, 0.0, 0.0}, 1.2,
                        {on_path}, partition_parameters).empty());
  CruisePredictedPath crossing;
  crossing.confidence = 1.0;
  crossing.poses = {{-1.0, 0.0, 0.0}, {1.0, 0.0, 0.0}};
  on_path.predicted_paths = {crossing};
  require(filter.filter(path, {0.0, 0.0, 0.0}, 1.3,
                        {on_path}).empty());
  object.classification = 1;
  require(!findRunOutCollision(path, {object}));
  object.classification = 7;
  object.pose.y = 5.0;
  object.predicted_paths.front().poses = {object.pose, object.pose};
  require(!findRunOutCollision(path, {object}));

  object.pose = {4.0, 2.0, 0.0};
  const auto without_path = createRunOutObjectsWithoutPath(path, {object});
  require(without_path.size() == 1);
  require(std::abs(without_path.front().pose.yaw_rad +
                   std::acos(-1.0) * 0.5) < 1e-9);
  require(without_path.front().predicted_paths.size() == 1);
  require(without_path.front().predicted_paths.front().poses.size() == 14);
  require(without_path.front().predicted_paths.front().poses[1].y < 2.0);
  const std::vector<RunOutPoint> points{
      {4.001, 1.0, 0.0}, {4.002, 2.0, 0.0},
      {4.002, 1.02, 5.0}, {8.0, 8.0, 0.0}};
  const auto from_points = createRunOutObjectsFromPoints(
      path, points,
      {{{3.0, 0.0, 0.0}, {5.0, 0.0, 0.0},
        {5.0, 3.0, 0.0}, {3.0, 3.0, 0.0}}});
  require(from_points.size() == 1);
  require(from_points.front().classification == 7);
  require(from_points.front().shape_type == 1);
  require(std::abs(from_points.front().pose.y - 1.01) < 1e-9);
  require(from_points.front().predicted_paths.front().poses.size() == 14);
}
