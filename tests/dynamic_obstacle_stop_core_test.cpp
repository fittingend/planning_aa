// SPDX-License-Identifier: Apache-2.0
// 동적 장애물의 종류·속도·방향 필터와 충돌 유지 시간을 검사합니다.

#include "planning/dynamic_obstacle_stop_core.hpp"

#include <cmath>
#include <stdexcept>

namespace {
void check(bool condition) {
  if (!condition) throw std::runtime_error("동적 장애물 정지 검사 실패");
}
}  // 네임스페이스

int main() {
  check(std::abs(planning::dynamicStopMinimumBrakingDistance(
      5.0, 0.0, -1.0, 1.0, -1.0) - 15.0) < 1e-9);
  check(planning::dynamicStopMinimumBrakingDistance(
      0.0, 0.0, -1.0, 1.0, -1.0) == 0.0);
  planning::Trajectory path(21);
  for (std::size_t i = 0; i < path.size(); ++i) path[i].x = i * 2.0;
  planning::TrajectoryPoint ego;
  ego.x = 2.0;
  planning::DynamicObstacleStopParameters parameters;
  planning::DynamicObject crossing;
  crossing.id = "교차차량";
  crossing.type = planning::DynamicObjectType::Car;
  crossing.x = 18.0;
  crossing.y = 1.0;
  crossing.yaw = 3.141592653589793 / 2.0;
  crossing.velocity = 2.0;
  crossing.length = 4.0;
  crossing.width = 2.0;
  auto filtered = planning::filterDynamicStopObjects(
      {crossing}, path, ego, 0.0, std::nullopt, parameters);
  check(filtered.size() == 1);
  auto footprint = planning::dynamicStopForwardFootprint(crossing, parameters);
  check(std::abs(footprint[0].y - 13.0) < 1e-12);
  crossing.type = planning::DynamicObjectType::Pedestrian;
  check(planning::filterDynamicStopObjects(
      {crossing}, path, ego, 0.0, std::nullopt, parameters).empty());
  crossing.type = planning::DynamicObjectType::Car;
  crossing.velocity = 0.1;
  check(planning::filterDynamicStopObjects(
      {crossing}, path, ego, 0.0, std::nullopt, parameters).empty());
  crossing.velocity = 2.0;
  crossing.y = 10.0;
  check(planning::filterDynamicStopObjects(
      {crossing}, path, ego, 0.0, std::nullopt, parameters).empty());
  crossing.y = 0.0;
  crossing.yaw = 3.141592653589793;
  check(planning::filterDynamicStopObjects(
      {crossing}, path, ego, 0.0, std::nullopt, parameters).empty());
  parameters.ignore_unavoidable_collisions = false;
  crossing.x = 30.0;
  const auto objects = planning::filterDynamicStopObjects(
      {crossing}, path, ego, 0.0, std::nullopt, parameters);
  check(objects.size() == 1);
  const auto collisions = planning::findDynamicStopCollisions(
      path, {ego.x, ego.y}, objects, parameters);
  check(collisions.size() == 1);
  check(std::abs(collisions.front().point.x - 18.0) < 1e-9);
  const auto stop = planning::dynamicStopPointFromCollision(
      path, {ego.x, ego.y}, collisions.front().point, 0.0, parameters);
  check(stop.has_value());
  check(std::abs(stop->x - 13.76) < 1e-9);

  planning::DynamicObstacleStopDecisionCore decisions(parameters);
  decisions.update(path, {{"가", {18.0, 0.0}}}, 0.0);
  check(!decisions.hasActiveDecision());
  decisions.update(path, {{"가", {17.0, 0.0}}}, 0.49);
  check(!decisions.hasActiveDecision());
  decisions.update(path, {{"가", {16.0, 0.0}}}, 0.5);
  check(decisions.hasActiveDecision());
  auto collision = decisions.earliestCollision(path, {2.0, 0.0});
  check(collision.has_value() && std::abs(collision->x - 16.0) < 1e-12);
  decisions.update(path, {}, 1.0);
  check(decisions.hasActiveDecision());
  decisions.update(path, {}, 1.5);
  check(!decisions.hasActiveDecision());

  planning::DynamicObstacleStopCore core(parameters);
  check(core.plan(path, ego, objects, 0.0, 0.0, 0.0).stop_points.empty());
  const auto planned = core.plan(path, ego, objects, 0.0, 0.0, 0.5);
  check(planned.stop_points.size() == 1);
  check(std::abs(planned.stop_points.front().x - 13.76) < 1e-9);
  check(core.plan(path, ego, {}, 0.0, 0.0, 1.0).stop_points.size() == 1);
  check(core.plan(path, ego, {}, 0.0, 0.0, 1.5).stop_points.empty());
}
