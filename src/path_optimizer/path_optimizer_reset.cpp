// Copyright 2023 TIER IV, Inc.
// SPDX-License-Identifier: Apache-2.0
// 경로 최적화기의 이전 상태 초기화 조건입니다.

#include "planning/path_optimizer_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace planning {
namespace {
double lateralOffset(const Trajectory &path, const TrajectoryPoint &ego) {
  double best_distance = std::numeric_limits<double>::infinity();
  double offset = 0.0;
  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    const double dx = path[i + 1].x - path[i].x;
    const double dy = path[i + 1].y - path[i].y;
    const double length2 = dx * dx + dy * dy;
    if (length2 <= 1e-12) continue;
    const double parameter = std::clamp(
        ((ego.x - path[i].x) * dx +
         (ego.y - path[i].y) * dy) / length2, 0.0, 1.0);
    const double closest_x = path[i].x + parameter * dx;
    const double closest_y = path[i].y + parameter * dy;
    const double distance = std::hypot(
        ego.x - closest_x, ego.y - closest_y);
    if (distance < best_distance) {
      best_distance = distance;
      offset = (dx * (ego.y - path[i].y) -
                dy * (ego.x - path[i].x)) /
               std::sqrt(length2);
    }
  }
  return offset;
}
}  // 익명 네임스페이스

bool requiresPathOptimizerReset(
    const PathOptimizerCoreInput &current,
    const std::optional<PathOptimizerCoreInput> &previous,
    const PathOptimizerResetParameters &p) {
  if (current.path.size() < 2 ||
      !std::isfinite(p.maximum_lateral_path_change) ||
      !std::isfinite(p.maximum_ego_movement) ||
      !std::isfinite(p.maximum_stopped_goal_movement) ||
      p.maximum_lateral_path_change < 0.0 ||
      p.maximum_ego_movement < 0.0 ||
      p.maximum_stopped_goal_movement < 0.0)
    throw std::invalid_argument("경로 최적화 초기화 입력이 유효하지 않습니다");
  if (!previous || previous->path.size() < 2) return true;
  if (std::abs(lateralOffset(previous->path, current.ego) -
               lateralOffset(current.path, current.ego)) >=
      p.maximum_lateral_path_change)
    return true;
  if (std::hypot(current.ego.x - previous->ego.x,
                 current.ego.y - previous->ego.y) >
      p.maximum_ego_movement)
    return true;
  if (std::abs(current.ego_speed) <= 1e-3 &&
      std::hypot(current.path.back().x - previous->path.back().x,
                 current.path.back().y - previous->path.back().y) >=
          p.maximum_stopped_goal_movement)
    return true;
  return false;
}

}  // planning 네임스페이스
