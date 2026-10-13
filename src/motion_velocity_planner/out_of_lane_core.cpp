// Copyright 2024 TIER IV, Inc.
// SPDX-License-Identifier: Apache-2.0
// 원본 차로 밖 주행 모듈의 시간 조건과 결정 유지 규칙입니다.

#include "planning/out_of_lane_core.hpp"

#include "planning/dynamic_obstacle_stop_core.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace planning {
namespace {
MotionVelocityPoint2D atArc(const Trajectory &path, double arc) {
  if (arc <= 0.0) return {path.front().x, path.front().y};
  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    const double length = std::hypot(path[i + 1].x - path[i].x,
                                     path[i + 1].y - path[i].y);
    if (arc <= length) {
      const double ratio = length < 1e-9 ? 0.0 : arc / length;
      return {path[i].x + ratio * (path[i + 1].x - path[i].x),
              path[i].y + ratio * (path[i + 1].y - path[i].y)};
    }
    arc -= length;
  }
  return {path.back().x, path.back().y};
}
}  // 익명 네임스페이스

OutOfLaneCollisionTime classifyOutOfLaneCollisionTimes(
    const std::vector<double> &object_arrival_times,
    const double ego_arrival_time, const OutOfLaneParameters &p) {
  if (!std::isfinite(ego_arrival_time))
    throw std::invalid_argument("차로 밖 주행 도착 시각이 유효하지 않습니다");
  OutOfLaneCollisionTime result;
  if (object_arrival_times.empty()) return result;
  for (const double time : object_arrival_times) {
    if (!std::isfinite(time))
      throw std::invalid_argument("객체 도착 시각이 유효하지 않습니다");
  }
  const auto limits = std::minmax_element(
      object_arrival_times.begin(), object_arrival_times.end());
  result.minimum_object_arrival = *limits.first;
  result.maximum_object_arrival = *limits.second;
  result.time_to_collision = ego_arrival_time >= *limits.first &&
      ego_arrival_time <= *limits.second ? 0.0 :
      std::min(std::abs(ego_arrival_time - *limits.first),
               std::abs(ego_arrival_time - *limits.second));
  result.avoid = p.time_mode == OutOfLaneTimeMode::TimeToCollision ?
      *result.time_to_collision <= p.time_to_collision_threshold :
      *result.minimum_object_arrival <= p.object_arrival_threshold;
  return result;
}

OutOfLaneDecisionCore::OutOfLaneDecisionCore(OutOfLaneParameters parameters)
    : parameters_(parameters) {}

void OutOfLaneDecisionCore::reset() { previous_.reset(); }

MotionVelocityPlan OutOfLaneDecisionCore::plan(
    const Trajectory &path, const MotionVelocityPoint2D ego,
    std::optional<MotionVelocityPoint2D> candidate,
    const double now_seconds) {
  if (path.size() < 2 || !std::isfinite(now_seconds))
    throw std::invalid_argument("차로 밖 주행 결정 입력이 유효하지 않습니다");
  if (previous_ && now_seconds - previous_time_ >
                       parameters_.minimum_decision_duration)
    previous_.reset();
  if (candidate) previous_time_ = now_seconds;
  if (candidate && previous_ &&
      dynamicStopArc(path, *previous_) < dynamicStopArc(path, *candidate))
    candidate = atArc(path, dynamicStopArc(path, *previous_));
  MotionVelocityPlan result;
  if (!candidate) return result;
  const double distance = dynamicStopArc(path, *candidate) -
                          dynamicStopArc(path, ego);
  previous_ = candidate;
  if (distance <= parameters_.stop_distance_threshold)
    result.stop_points.push_back(*candidate);
  else
    result.slowdowns.push_back({*candidate, *candidate,
                                parameters_.slowdown_velocity});
  return result;
}

}  // planning 네임스페이스
