// Copyright 2024 TIER IV, Inc.
// SPDX-License-Identifier: Apache-2.0
// 차로 밖 주행 충돌 판단에 앞선 객체 예측 경로 필터입니다.

#include "planning/out_of_lane_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace planning {
namespace {
double squaredDistance(MotionVelocityPoint2D a, MotionVelocityPoint2D b) {
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return dx * dx + dy * dy;
}

double closestPathOffset(const Trajectory &path,
                         MotionVelocityPoint2D point) {
  double best_distance = std::numeric_limits<double>::infinity();
  double best_arc = 0.0;
  double arc = 0.0;
  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    const double dx = path[i + 1].x - path[i].x;
    const double dy = path[i + 1].y - path[i].y;
    const double length2 = dx * dx + dy * dy;
    if (length2 < 1e-12) continue;
    const double parameter =
        ((point.x - path[i].x) * dx + (point.y - path[i].y) * dy) /
        length2;
    const double projected = std::clamp(parameter, 0.0, 1.0);
    const MotionVelocityPoint2D projection{
        path[i].x + projected * dx, path[i].y + projected * dy};
    const double distance = squaredDistance(point, projection);
    if (distance < best_distance) {
      best_distance = distance;
      best_arc = arc + parameter * std::sqrt(length2);
    }
    arc += std::sqrt(length2);
  }
  return best_arc;
}

double lateralPathDistance(const std::vector<TrajectoryPoint> &path,
                           MotionVelocityPoint2D point) {
  double minimum = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    const double dx = path[i + 1].x - path[i].x;
    const double dy = path[i + 1].y - path[i].y;
    const double length2 = dx * dx + dy * dy;
    if (length2 < 1e-12) continue;
    const double parameter = std::clamp(
        ((point.x - path[i].x) * dx + (point.y - path[i].y) * dy) /
            length2,
        0.0, 1.0);
    const double cross =
        (point.x - path[i].x) * dy -
        (point.y - path[i].y) * dx;
    const double distance = std::abs(cross) / std::sqrt(length2);
    // 선분 바깥의 끝점은 거리 대신 끝점까지의 거리로 판정합니다.
    const MotionVelocityPoint2D projection{
        path[i].x + parameter * dx,
        path[i].y + parameter * dy};
    const double bounded_distance =
        parameter == 0.0 || parameter == 1.0
            ? std::sqrt(squaredDistance(point, projection))
            : distance;
    minimum = std::min(minimum, bounded_distance);
  }
  return minimum;
}

std::vector<TrajectoryPoint> removeOverlap(
    const std::vector<TrajectoryPoint> &path) {
  std::vector<TrajectoryPoint> result;
  for (const auto &point : path) {
    if (result.empty() ||
        std::hypot(point.x - result.back().x,
                   point.y - result.back().y) > 1e-3)
      result.push_back(point);
  }
  return result;
}
}  // 익명 네임스페이스

std::vector<OutOfLanePredictedPath> filterOutOfLanePredictedObjects(
    const Trajectory &ego_path, const MotionVelocityPoint2D ego_position,
    const std::vector<OutOfLaneTrackedObject> &objects,
    const std::vector<OutOfLaneStopLine> &stop_lines,
    const OutOfLaneObjectFilterParameters &p) {
  if (ego_path.size() < 2 || !std::isfinite(p.minimum_confidence) ||
      p.minimum_confidence < 0.0)
    throw std::invalid_argument("차로 밖 주행 객체 필터 입력이 유효하지 않습니다");
  std::vector<OutOfLanePredictedPath> filtered;
  for (const auto &object : objects) {
    if (object.is_pedestrian) continue;
    if (p.ignore_behind_ego &&
        closestPathOffset(ego_path, object.initial_position) < 0.0)
      continue;
    const double crossing_width = object.width / 2.0 +
        std::max(p.left_offset + p.extra_left_offset,
                 p.right_offset + p.extra_right_offset);
    for (auto prediction : object.predicted_paths) {
      if (prediction.confidence < p.minimum_confidence) continue;
      auto unique_path = removeOverlap(prediction.poses);
      if (unique_path.size() <= 1) continue;
      if (lateralPathDistance(unique_path, ego_position) <= crossing_width)
        continue;
      if (p.cut_at_stop_lines)
        prediction = cutOutOfLanePredictedPathAtStopLines(
            std::move(prediction), stop_lines, prediction.object_length);
      if (!prediction.poses.empty()) filtered.push_back(std::move(prediction));
    }
  }
  return filtered;
}

}  // planning 네임스페이스
