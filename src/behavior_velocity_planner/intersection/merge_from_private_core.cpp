// Copyright 2020 Tier IV, Inc.
// SPDX-License-Identifier: Apache-2.0
// 사유 도로 합류 정지점을 만들고 정차 후 출발 상태를 유지합니다.

#include "planning/merge_from_private_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace planning {
namespace {

double projectArc(const Trajectory &path, double x, double y) {
  if (path.size() < 2) return 0.0;
  double arc = 0.0;
  double best_arc = 0.0;
  double best_distance = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0; index + 1 < path.size(); ++index) {
    const double dx = path[index + 1].x - path[index].x;
    const double dy = path[index + 1].y - path[index].y;
    const double length2 = dx * dx + dy * dy;
    if (length2 <= 1.0e-12) continue;
    const double ratio = std::clamp(
        ((x - path[index].x) * dx + (y - path[index].y) * dy) /
            length2,
        0.0, 1.0);
    const double distance = std::hypot(
        x - path[index].x - ratio * dx,
        y - path[index].y - ratio * dy);
    const double step = std::sqrt(length2);
    if (distance < best_distance) {
      best_distance = distance;
      best_arc = arc + ratio * step;
    }
    arc += step;
  }
  return best_arc;
}

}  // 익명 네임스페이스

MergeFromPrivateResult MergeFromPrivateCore::apply(
    const Trajectory &path,
    const std::vector<CrosswalkBounds> &conflicting_lanes,
    double ego_x, double ego_y, double ego_velocity_mps,
    double now_s, double minimum_conflict_arc_m) {
  for (const double value : {
           ego_x, ego_y, ego_velocity_mps, now_s,
           minimum_conflict_arc_m}) {
    if (!std::isfinite(value)) {
      throw std::invalid_argument(
          "사유 도로 합류 입력에는 유한한 값이 필요합니다");
    }
  }
  if (minimum_conflict_arc_m < 0.0) {
    throw std::invalid_argument("사유 도로 충돌 시작 거리가 잘못되었습니다");
  }
  MergeFromPrivateResult result;
  result.trajectory = path;
  if (go_) return result;
  IntersectionStuckParameters parameters;
  parameters.minimum_conflict_arc_m = minimum_conflict_arc_m;
  const auto stopped = planIntersectionConflictStop(
      path, conflicting_lanes, ego_x, ego_y, parameters);
  if (!stopped.stop_arc_m) return result;
  result.trajectory = stopped.trajectory;
  result.stop_arc_m = stopped.stop_arc_m;
  result.stopped = true;
  if (ego_velocity_mps < 1.0e-3) {
    if (!stopped_since_s_ || now_s < *stopped_since_s_) {
      stopped_since_s_ = now_s;
    }
  } else {
    stopped_since_s_.reset();
  }
  if (stopped_since_s_ && now_s - *stopped_since_s_ >= 1.0 &&
      *stopped.stop_arc_m - projectArc(path, ego_x, ego_y) < 1.0) {
    go_ = true;
  }
  return result;
}

}  // planning 네임스페이스
