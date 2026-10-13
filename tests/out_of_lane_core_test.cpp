// SPDX-License-Identifier: Apache-2.0
// 겹침 영역의 도착 시각과 정지·감속 유지 규칙을 검사합니다.

#include "planning/out_of_lane_core.hpp"

#include <cmath>
#include <stdexcept>

namespace {
void check(bool condition) {
  if (!condition) throw std::runtime_error("차로 밖 주행 검사 실패");
}
}  // 익명 네임스페이스

int main() {
  planning::OutOfLaneParameters parameters;
  auto times = planning::classifyOutOfLaneCollisionTimes(
      {2.0, 3.0, 4.0}, 3.5, parameters);
  check(times.avoid && *times.time_to_collision == 0.0);
  times = planning::classifyOutOfLaneCollisionTimes(
      {2.0, 3.0, 4.0}, 5.1, parameters);
  check(!times.avoid && std::abs(*times.time_to_collision - 1.1) < 1e-12);
  parameters.time_mode = planning::OutOfLaneTimeMode::ArrivalThreshold;
  times = planning::classifyOutOfLaneCollisionTimes(
      {4.0, 7.0}, 20.0, parameters);
  check(times.avoid);
  times = planning::classifyOutOfLaneCollisionTimes({}, 2.0, parameters);
  check(!times.avoid && !times.time_to_collision);

  planning::Trajectory path(21);
  for (std::size_t i = 0; i < path.size(); ++i) path[i].x = i * 2.0;
  planning::OutOfLaneDecisionCore core(parameters);
  auto result = core.plan(path, {0.0, 0.0}, {{20.0, 0.0}}, 0.0);
  check(result.slowdowns.size() == 1 && result.stop_points.empty());
  result = core.plan(path, {10.0, 0.0}, {{24.0, 0.0}}, 0.5);
  check(result.stop_points.size() == 1 && result.stop_points[0].x == 20.0);
  result = core.plan(path, {10.0, 0.0}, {{24.0, 0.0}}, 2.0);
  check(result.stop_points.size() == 1 && result.stop_points[0].x == 24.0);
  result = core.plan(path, {10.0, 0.0}, std::nullopt, 2.1);
  check(result.stop_points.empty() && result.slowdowns.empty());

  planning::Trajectory geometry_path(31);
  std::vector<double> arrival(31);
  for (std::size_t i = 0; i < geometry_path.size(); ++i) {
    geometry_path[i].x = static_cast<double>(i);
    arrival[i] = static_cast<double>(i) / 10.0;
  }
  planning::OutOfLanePolygon other_lane{
      {20.0, 0.7}, {30.0, 0.7}, {30.0, 3.0}, {20.0, 3.0}};
  planning::OutOfLanePredictedPath object;
  object.time_step = 0.5;
  object.object_length = 4.0;
  object.object_width = 2.0;
  planning::TrajectoryPoint object_pose;
  object_pose.x = 20.0;
  object_pose.y = 1.0;
  object.poses.push_back(object_pose);
  const auto candidate = planning::findOutOfLaneSlowdownCandidate(
      geometry_path, arrival, {other_lane}, {object}, 0.0);
  check(candidate && candidate->x >= 14.0 && candidate->x < 17.0);
  planning::OutOfLaneDecisionCore geometry_decision;
  const auto geometry_plan = geometry_decision.plan(
      geometry_path, {2.0, 0.0}, candidate, 1.0);
  check(geometry_plan.stop_points.size() == 1 &&
        geometry_plan.slowdowns.empty());
  check(!planning::findOutOfLaneSlowdownCandidate(
      geometry_path, arrival, {other_lane}, {}, 0.0));

  planning::OutOfLanePredictedPath crossing;
  for (std::size_t i = 0; i < 10; ++i) {
    planning::TrajectoryPoint pose;
    pose.x = static_cast<double>(i);
    crossing.poses.push_back(pose);
  }
  planning::OutOfLaneStopLine line;
  line.vertices = {{5.5, -3.0}, {5.5, 3.0}};
  line.lane_polygon = {{0.0, -2.0}, {6.0, -2.0},
                       {6.0, 2.0}, {0.0, 2.0}};
  const auto cut = planning::cutOutOfLanePredictedPathAtStopLines(
      crossing, {line}, 2.0);
  check(cut.poses.size() == 4 && cut.poses.back().x == 3.0);
  planning::OutOfLaneStopLine unrelated_line;
  unrelated_line.vertices = {{2.5, -3.0}, {2.5, 3.0}};
  unrelated_line.lane_polygon = {{20.0, -2.0}, {30.0, -2.0},
                                 {30.0, 2.0}, {20.0, 2.0}};
  const auto no_cut = planning::cutOutOfLanePredictedPathAtStopLines(
      crossing, {unrelated_line}, 2.0);
  check(no_cut.poses.size() == crossing.poses.size());

  planning::OutOfLaneTrackedObject tracked;
  tracked.initial_position = {5.0, 3.0};
  tracked.width = 2.0;
  crossing.object_length = 2.0;
  for (auto &pose : crossing.poses) pose.y = 3.0;
  tracked.predicted_paths = {crossing};
  const auto selected = planning::filterOutOfLanePredictedObjects(
      geometry_path, {0.0, 0.0}, {tracked}, {}, {});
  check(selected.size() == 1 && selected.front().poses.size() == 10);
  planning::OutOfLaneStopLine object_line;
  object_line.vertices = {{5.5, 2.0}, {5.5, 4.0}};
  object_line.lane_polygon = {{0.0, 2.0}, {6.0, 2.0},
                              {6.0, 4.0}, {0.0, 4.0}};
  const auto red_filtered = planning::filterOutOfLanePredictedObjects(
      geometry_path, {0.0, 0.0}, {tracked}, {object_line}, {});
  check(red_filtered.size() == 1 && red_filtered.front().poses.size() == 4);
  tracked.is_pedestrian = true;
  check(planning::filterOutOfLanePredictedObjects(
      geometry_path, {0.0, 0.0}, {tracked}, {}, {}).empty());
  tracked.is_pedestrian = false;
  tracked.initial_position.x = -2.0;
  check(planning::filterOutOfLanePredictedObjects(
      geometry_path, {0.0, 0.0}, {tracked}, {}, {}).empty());
  tracked.initial_position.x = 5.0;
  for (auto &pose : tracked.predicted_paths.front().poses) pose.y = 0.0;
  check(planning::filterOutOfLanePredictedObjects(
      geometry_path, {0.0, 0.0}, {tracked}, {}, {}).empty());
  for (auto &pose : tracked.predicted_paths.front().poses) pose.y = 3.0;
  tracked.predicted_paths.front().confidence = 0.05;
  check(planning::filterOutOfLanePredictedObjects(
      geometry_path, {0.0, 0.0}, {tracked}, {}, {}).empty());
}
