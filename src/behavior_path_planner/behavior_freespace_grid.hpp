// SPDX-License-Identifier: Apache-2.0
// 자유공간 주차의 점유 셀 변환과 차량 외곽 충돌 검사를 공유합니다.
#pragma once

#include "planning/behavior_pull_over_core.hpp"

#include <algorithm>
#include <cmath>
#include <optional>
#include <utility>

namespace planning::detail {

inline std::optional<std::pair<int, int>> cellOf(
    const CrosswalkOccupancyGrid &grid, double x, double y) {
  const double dx = x - grid.origin_x;
  const double dy = y - grid.origin_y;
  const double cosine = std::cos(grid.origin_yaw_rad);
  const double sine = std::sin(grid.origin_yaw_rad);
  const int ix = static_cast<int>(std::floor(
      (cosine * dx + sine * dy) / grid.resolution_m));
  const int iy = static_cast<int>(std::floor(
      (-sine * dx + cosine * dy) / grid.resolution_m));
  if (ix < 0 || iy < 0 ||
      ix >= static_cast<int>(grid.width) ||
      iy >= static_cast<int>(grid.height)) return std::nullopt;
  return std::pair<int, int>{ix, iy};
}

inline bool footprintFree(const CrosswalkOccupancyGrid &grid,
                   const FreespacePullOverParameters &p,
                   double x, double y, double yaw) {
  const double margin = p.vehicle_shape_margin_m * 0.5;
  const double length_step = std::min(0.5, grid.resolution_m * 0.5);
  const double width_step = std::min(0.5, grid.resolution_m * 0.5);
  const int length_samples = std::max(1, static_cast<int>(std::ceil(
      (p.front_m + p.rear_m + 2.0 * margin) /
      length_step)));
  const int width_samples = std::max(1, static_cast<int>(std::ceil(
      2.0 * (p.half_width_m + margin) /
      width_step)));
  const double front = p.front_m + margin;
  const double rear = p.rear_m + margin;
  const double half_width = p.half_width_m + margin;
  const double cosine = std::cos(yaw);
  const double sine = std::sin(yaw);
  for (int along = 0; along <= length_samples; ++along) {
    const double longitudinal = -rear +
        (front + rear) * along / length_samples;
    for (int across = 0; across <= width_samples; ++across) {
      const double lateral = -half_width +
          2.0 * half_width * across / width_samples;
      const auto cell = cellOf(grid,
          x + longitudinal * cosine - lateral * sine,
          y + longitudinal * sine + lateral * cosine);
      if (!cell) return false;
      const auto value = grid.values[
          static_cast<std::size_t>(cell->second) * grid.width +
          static_cast<std::size_t>(cell->first)];
      if (value < 0 || value >= p.obstacle_threshold) return false;
    }
  }
  return true;
}

}  // planning::detail 네임스페이스
