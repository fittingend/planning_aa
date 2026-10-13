// Copyright 2023-2024 TIER IV, Inc.
// SPDX-License-Identifier: Apache-2.0
// 원본 순항 계획의 점군 복셀 축소, 유클리드 군집 및 충돌점 선택입니다.

#include "planning/obstacle_cruise_pointcloud.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <queue>
#include <stdexcept>
#include <tuple>

namespace planning {
namespace {
using Cell = std::tuple<long long, long long, long long>;

void validate(const CruisePointcloudParameters &p) {
  const double values[] = {p.voxel_x_m, p.voxel_y_m, p.voxel_z_m,
                           p.cluster_tolerance_m, p.vehicle_width_m,
                           p.stop_lateral_margin_m, p.slowdown_lateral_margin_m};
  if (!std::all_of(std::begin(values), std::end(values),
                   [](double value) { return std::isfinite(value); }) ||
      p.voxel_x_m <= 0.0 || p.voxel_y_m <= 0.0 || p.voxel_z_m <= 0.0 ||
      p.cluster_tolerance_m <= 0.0 || p.minimum_cluster_size == 0 ||
      p.maximum_cluster_size < p.minimum_cluster_size ||
      p.vehicle_width_m < 0.0 || p.stop_lateral_margin_m < 0.0 ||
      p.slowdown_lateral_margin_m < 0.0)
    throw std::invalid_argument("순항 점군 설정이 유효하지 않습니다");
}

long long cellIndex(const double value, const double size) {
  const double index = std::floor(value / size);
  if (!std::isfinite(index) ||
      std::abs(index) > static_cast<double>(std::numeric_limits<long long>::max() / 2))
    throw std::invalid_argument("점군 좌표가 복셀 범위를 벗어났습니다");
  return static_cast<long long>(index);
}

Cell cellOf(const CruisePoint3D &point, const double x,
            const double y, const double z) {
  return {cellIndex(point.x, x), cellIndex(point.y, y),
          cellIndex(point.z, z)};
}

std::size_t nearestPointIndex(const Trajectory &path,
                              const CruisePoint3D &point) {
  std::size_t best = 0;
  double distance = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < path.size(); ++i) {
    const double current = std::hypot(path[i].x - point.x,
                                      path[i].y - point.y);
    if (current < distance) { distance = current; best = i; }
  }
  return best;
}

double lateralOffset(const Trajectory &path, const CruisePoint3D &point) {
  double best_distance = std::numeric_limits<double>::infinity();
  double offset = 0.0;
  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    const double dx = path[i + 1].x - path[i].x;
    const double dy = path[i + 1].y - path[i].y;
    const double length_squared = dx * dx + dy * dy;
    if (length_squared < 1e-12) continue;
    const double ratio = std::clamp(
        ((point.x - path[i].x) * dx + (point.y - path[i].y) * dy) /
            length_squared,
        0.0, 1.0);
    const double x = path[i].x + ratio * dx;
    const double y = path[i].y + ratio * dy;
    const double distance = std::hypot(point.x - x, point.y - y);
    if (distance < best_distance) {
      best_distance = distance;
      offset = (dx * (point.y - path[i].y) -
                dy * (point.x - path[i].x)) / std::sqrt(length_squared);
    }
  }
  return offset;
}
}  // 익명 네임스페이스

std::vector<CruisePoint3D> voxelizeCruisePointcloud(
    const std::vector<CruisePoint3D> &points,
    const CruisePointcloudParameters &p) {
  validate(p);
  struct Accumulator { double x = 0.0, y = 0.0, z = 0.0; std::size_t n = 0; };
  std::map<Cell, Accumulator> cells;
  for (const auto &point : points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z)) continue;
    auto &sum = cells[cellOf(point, p.voxel_x_m, p.voxel_y_m, p.voxel_z_m)];
    sum.x += point.x; sum.y += point.y; sum.z += point.z; ++sum.n;
  }
  std::vector<CruisePoint3D> result;
  result.reserve(cells.size());
  for (const auto &[_, sum] : cells)
    result.push_back({sum.x / sum.n, sum.y / sum.n, sum.z / sum.n});
  return result;
}

std::vector<std::vector<std::size_t>> clusterCruisePointcloud(
    const std::vector<CruisePoint3D> &points,
    const CruisePointcloudParameters &p) {
  validate(p);
  std::map<Cell, std::vector<std::size_t>> cells;
  for (std::size_t i = 0; i < points.size(); ++i) {
    const auto &point = points[i];
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z)) continue;
    cells[cellOf(point, p.cluster_tolerance_m,
                 p.cluster_tolerance_m, p.cluster_tolerance_m)].push_back(i);
  }
  std::vector<bool> visited(points.size(), false);
  std::vector<std::vector<std::size_t>> result;
  const double threshold_squared = p.cluster_tolerance_m * p.cluster_tolerance_m;
  for (std::size_t start = 0; start < points.size(); ++start) {
    if (visited[start] || !std::isfinite(points[start].x) ||
        !std::isfinite(points[start].y) || !std::isfinite(points[start].z))
      continue;
    visited[start] = true;
    std::queue<std::size_t> pending;
    pending.push(start);
    std::vector<std::size_t> cluster;
    while (!pending.empty()) {
      const std::size_t current = pending.front();
      pending.pop();
      cluster.push_back(current);
      const auto [cx, cy, cz] = cellOf(
          points[current], p.cluster_tolerance_m,
          p.cluster_tolerance_m, p.cluster_tolerance_m);
      for (long long dx = -1; dx <= 1; ++dx)
        for (long long dy = -1; dy <= 1; ++dy)
          for (long long dz = -1; dz <= 1; ++dz) {
            const auto cell = cells.find({cx + dx, cy + dy, cz + dz});
            if (cell == cells.end()) continue;
            for (const auto index : cell->second) {
              if (visited[index]) continue;
              const double x = points[index].x - points[current].x;
              const double y = points[index].y - points[current].y;
              const double z = points[index].z - points[current].z;
              if (x * x + y * y + z * z > threshold_squared) continue;
              visited[index] = true;
              pending.push(index);
            }
          }
    }
    if (cluster.size() >= p.minimum_cluster_size &&
        cluster.size() <= p.maximum_cluster_size)
      result.push_back(std::move(cluster));
  }
  return result;
}

std::vector<CruisePointcloudObstacle> convertCruisePointcloudToObstacles(
    const std::vector<CruisePoint3D> &transformed_points,
    const Trajectory &trajectory, const std::size_t ego_index,
    const CruisePointcloudParameters &p) {
  validate(p);
  if (trajectory.size() < 2 || ego_index >= trajectory.size())
    throw std::invalid_argument("순항 점군 경로가 유효하지 않습니다");
  const auto points = voxelizeCruisePointcloud(transformed_points, p);
  const auto clusters = clusterCruisePointcloud(points, p);
  std::vector<double> arc(trajectory.size(), 0.0);
  for (std::size_t i = 1; i < trajectory.size(); ++i)
    arc[i] = arc[i - 1] + std::hypot(
        trajectory[i].x - trajectory[i - 1].x,
        trajectory[i].y - trajectory[i - 1].y);
  std::vector<CruisePointcloudObstacle> result;
  const double max_margin = std::max(p.stop_lateral_margin_m,
                                     p.slowdown_lateral_margin_m);
  for (const auto &cluster : clusters) {
    CruisePointcloudObstacle obstacle;
    obstacle.cluster_size = cluster.size();
    obstacle.distance_from_ego_m = std::numeric_limits<double>::infinity();
    obstacle.signed_lateral_distance_m =
        std::numeric_limits<double>::infinity();
    double stop_distance = std::numeric_limits<double>::infinity();
    double slow_front_distance = std::numeric_limits<double>::infinity();
    double slow_back_distance = std::numeric_limits<double>::min();
    for (const auto index : cluster) {
      const auto &point = points[index];
      const double lateral = lateralOffset(trajectory, point);
      const double margin = std::abs(lateral) - p.vehicle_width_m;
      if (margin >= max_margin) continue;
      const std::size_t nearest = nearestPointIndex(trajectory, point);
      const double distance = arc[nearest] - arc[ego_index];
      if (distance < 0.0) continue;
      obstacle.distance_from_ego_m =
          std::min(obstacle.distance_from_ego_m, distance);
      obstacle.signed_lateral_distance_m =
          std::min(obstacle.signed_lateral_distance_m, lateral);
      if (margin < p.stop_lateral_margin_m) {
        if (distance < stop_distance) {
          stop_distance = distance;
          obstacle.stop_collision_point = point;
        }
      } else if (margin < p.slowdown_lateral_margin_m) {
        if (distance < slow_front_distance) {
          slow_front_distance = distance;
          obstacle.slowdown_front_collision_point = point;
        } else if (distance > slow_back_distance) {
          slow_back_distance = distance;
          obstacle.slowdown_back_collision_point = point;
        }
      }
    }
    if (obstacle.stop_collision_point ||
        obstacle.slowdown_front_collision_point)
      result.push_back(std::move(obstacle));
  }
  return result;
}

}  // planning 네임스페이스
