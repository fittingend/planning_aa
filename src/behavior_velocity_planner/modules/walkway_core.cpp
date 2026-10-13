// Copyright 2020 Tier IV, Inc.
// SPDX-License-Identifier: Apache-2.0
// 원본 WalkwayModule의 접근·정차·통과 순서를 유지합니다.

#include "planning/walkway_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace planning {
namespace {

double egoArc(const Trajectory &path, double x, double y) {
  double best_distance = std::numeric_limits<double>::infinity();
  double best_arc = 0.0;
  double arc = 0.0;
  for (std::size_t index = 0; index + 1 < path.size(); ++index) {
    const double dx = path[index + 1].x - path[index].x;
    const double dy = path[index + 1].y - path[index].y;
    const double length2 = dx * dx + dy * dy;
    if (length2 <= 1.0e-12) continue;
    const double ratio = std::clamp(
        ((x - path[index].x) * dx + (y - path[index].y) * dy) / length2,
        0.0, 1.0);
    const double offset_x = x - path[index].x - ratio * dx;
    const double offset_y = y - path[index].y - ratio * dy;
    const double distance2 = offset_x * offset_x + offset_y * offset_y;
    if (distance2 < best_distance) {
      best_distance = distance2;
      best_arc = arc + ratio * std::sqrt(length2);
    }
    arc += std::sqrt(length2);
  }
  return best_arc;
}

}  // 익명 네임스페이스

WalkwayCore::WalkwayCore(std::vector<CrosswalkPoint> polygon,
                         std::vector<StopLineSegment> stop_lines,
                         WalkwayParameters parameters)
    : polygon_(std::move(polygon)), stop_lines_(std::move(stop_lines)),
      parameters_(parameters) {
  if (!std::isfinite(parameters.stop_distance_m) ||
      !std::isfinite(parameters.front_offset_m) ||
      !std::isfinite(parameters.stop_duration_s) ||
      parameters.stop_distance_m < 0.0 ||
      parameters.front_offset_m < 0.0 ||
      parameters.stop_duration_s < 0.0) {
    throw std::invalid_argument("보행로 정지 설정이 잘못되었습니다");
  }
}

WalkwayResult WalkwayCore::apply(const Trajectory &path, double ego_x,
                                 double ego_y, bool vehicle_stopped,
                                 double now_s) {
  if (!std::isfinite(ego_x) || !std::isfinite(ego_y) ||
      !std::isfinite(now_s)) {
    throw std::invalid_argument("차량 위치와 시각에는 유한한 값이 필요합니다");
  }
  WalkwayResult result;
  result.trajectory = path;
  result.state = state_;
  const auto crossing = planCrosswalkDefaultStop(
      path, polygon_, {parameters_.stop_distance_m, parameters_.front_offset_m});
  if (!crossing.entry_arc_m) return result;
  if (state_ == WalkwayState::Approach) {
    result.trajectory = crossing.trajectory;
    result.stop_arc_m = crossing.stop_arc_m;
    for (const auto &line : stop_lines_) {
      StopLineParameters settings;
      settings.front_offset_m = parameters_.front_offset_m;
      settings.line_extension_m = 0.0;
      StopLineCore stop_line(line, settings);
      const auto candidate = stop_line.apply(path, ego_x, ego_y, false, now_s);
      if (!candidate.stop_arc_m) continue;
      result.trajectory = candidate.trajectory;
      result.stop_arc_m = candidate.stop_arc_m;
      break;
    }
    if (result.stop_arc_m) {
      const double distance = *result.stop_arc_m - egoArc(path, ego_x, ego_y);
      if (distance < 1.0 && vehicle_stopped) {
        if (!stopped_since_s_) stopped_since_s_ = now_s;
        if (now_s - *stopped_since_s_ >= parameters_.stop_duration_s) {
          state_ = WalkwayState::Stop;
        }
      } else {
        stopped_since_s_.reset();
      }
    }
  } else if (state_ == WalkwayState::Stop && vehicle_stopped) {
    state_ = WalkwayState::Surpassed;
  }
  result.state = state_;
  return result;
}

}  // planning 네임스페이스
