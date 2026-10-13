// SPDX-License-Identifier: Apache-2.0
// 점군 복셀·군집과 정지·감속 충돌점 선택을 검사합니다.

#include "planning/obstacle_cruise_pointcloud.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

namespace {
void check(bool condition) {
  if (!condition) throw std::runtime_error("순항 점군 검사 실패");
}
}  // 익명 네임스페이스

int main() {
  planning::Trajectory path(11);
  for (std::size_t i = 0; i < path.size(); ++i)
    path[i].x = static_cast<double>(i);
  const std::vector<planning::CruisePoint3D> points{
      {3.01, 1.0, 0.0}, {3.02, 1.0, 0.0},
      {5.0, 2.5, 0.0}, {6.0, 2.5, 0.0},
      {8.0, 4.0, 0.0},
      {9.0, 0.0, std::numeric_limits<double>::quiet_NaN()}};
  const auto voxels = planning::voxelizeCruisePointcloud(points);
  check(voxels.size() == 4);
  const auto clusters = planning::clusterCruisePointcloud(voxels);
  check(clusters.size() == 3);
  const auto obstacles = planning::convertCruisePointcloudToObstacles(
      points, path, 0);
  check(obstacles.size() == 2);
  bool found_stop = false, found_slow = false;
  for (const auto &obstacle : obstacles) {
    if (obstacle.stop_collision_point) {
      check(std::abs(obstacle.stop_collision_point->x - 3.015) < 1e-9);
      check(!obstacle.slowdown_front_collision_point);
      check(std::abs(obstacle.distance_from_ego_m - 3.0) < 1e-9);
      found_stop = true;
    } else {
      check(obstacle.slowdown_front_collision_point.has_value());
      check(obstacle.slowdown_back_collision_point.has_value());
      check(std::abs(obstacle.slowdown_front_collision_point->x - 5.0) < 1e-9);
      check(std::abs(obstacle.slowdown_back_collision_point->x - 6.0) < 1e-9);
      found_slow = true;
    }
  }
  check(found_stop && found_slow);
  planning::CruisePointcloudParameters parameters;
  parameters.minimum_cluster_size = 2;
  check(planning::convertCruisePointcloudToObstacles(
      points, path, 0, parameters).size() == 1);
  parameters.voxel_x_m = 0.0;
  bool rejected = false;
  try { planning::voxelizeCruisePointcloud(points, parameters); }
  catch (const std::invalid_argument &) { rejected = true; }
  check(rejected);
}
