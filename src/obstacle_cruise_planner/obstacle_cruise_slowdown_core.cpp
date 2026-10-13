// Copyright 2022-2024 TIER IV, Inc.
// SPDX-License-Identifier: Apache-2.0
// 원본 저속 통과 계획의 거리·속도 조건을 독립 계산으로 옮깁니다.

#include "planning/obstacle_cruise_slowdown_core.hpp"

#include "planning/dynamic_obstacle_stop_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>
#include <utility>

namespace planning {
namespace {
double lowpass(double current, double previous, double gain) {
  return gain * previous + (1.0 - gain) * current;
}

std::optional<MotionVelocityPoint2D> pointAtArc(
    const Trajectory &path, double arc) {
  if (arc < 0.0) return std::nullopt;
  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    const double length = std::hypot(path[i + 1].x - path[i].x,
                                     path[i + 1].y - path[i].y);
    if (arc > length) { arc -= length; continue; }
    const double ratio = length < 1e-9 ? 0.0 : arc / length;
    return MotionVelocityPoint2D{
        path[i].x + ratio * (path[i + 1].x - path[i].x),
        path[i].y + ratio * (path[i + 1].y - path[i].y)};
  }
  return std::nullopt;
}

double jerkFeasibleVelocity(double jerk, double acceleration,
                            double initial_acceleration,
                            double initial_velocity, double distance) {
  if (distance <= 0.0) return initial_velocity;
  const double transition_time =
      (acceleration - initial_acceleration) / jerk;
  const auto traveled = [&](double time) {
    return jerk * time * time * time / 6.0 +
        initial_acceleration * time * time / 2.0 +
        initial_velocity * time;
  };
  const auto speed = [&](double time) {
    return jerk * time * time / 2.0 +
        initial_acceleration * time + initial_velocity;
  };
  const double transition_distance = traveled(transition_time);
  if (distance < transition_distance) {
    double lower = 0.0, upper = transition_time;
    for (int i = 0; i < 80; ++i) {
      const double middle = 0.5 * (lower + upper);
      if (traveled(middle) > distance) upper = middle;
      else lower = middle;
    }
    return speed(0.5 * (lower + upper));
  }
  const double first_speed = speed(transition_time);
  const double remaining = distance - transition_distance;
  const double discriminant = first_speed * first_speed +
                              2.0 * acceleration * remaining;
  if (discriminant <= 0.0) return 0.0;
  const double constant_time =
      (-first_speed + std::sqrt(discriminant)) / acceleration;
  return first_speed + acceleration * constant_time;
}
}  // 익명 네임스페이스

ObstacleCruiseSlowdownCore::ObstacleCruiseSlowdownCore(
    ObstacleCruiseSlowdownParameters parameters) : parameters_(parameters) {}

void ObstacleCruiseSlowdownCore::reset() { previous_.clear(); }

ObstacleCruiseSlowdownResult ObstacleCruiseSlowdownCore::plan(
    const Trajectory &path, const MotionVelocityPoint2D ego,
    const double ego_velocity, const double ego_acceleration,
    const std::vector<ObstacleCruiseSlowdownCandidate> &obstacles) {
  if (path.size() < 2 || !std::isfinite(ego_velocity) ||
      !std::isfinite(ego_acceleration) ||
      parameters_.maximum_lateral_margin <= parameters_.minimum_lateral_margin ||
      parameters_.slowdown_acceleration >= 0.0 ||
      parameters_.slowdown_jerk >= 0.0)
    throw std::invalid_argument("장애물 감속 계획 입력이 유효하지 않습니다");
  ObstacleCruiseSlowdownResult result;
  result.trajectory = path;
  std::unordered_map<std::string, Previous> updated;
  const auto &p = parameters_;
  const double ego_arc = dynamicStopArc(path, ego);
  for (const auto &obstacle : obstacles) {
    const auto found = previous_.find(obstacle.id);
    const Previous *previous = found == previous_.end() ? nullptr : &found->second;
    const double object_speed = std::hypot(
        obstacle.longitudinal_velocity, obstacle.lateral_velocity);
    const bool moving = previous ?
        object_speed > p.moving_speed_threshold +
            (previous->moving ? -p.moving_hysteresis : p.moving_hysteresis) :
        object_speed > p.moving_speed_threshold;
    const double lateral = previous ? lowpass(
        obstacle.lateral_distance, previous->lateral_distance,
        p.lateral_filter_gain) : obstacle.lateral_distance;
    const double ratio = std::clamp(
        (std::abs(lateral) - p.minimum_lateral_margin) /
            (p.maximum_lateral_margin - p.minimum_lateral_margin),
        0.0, 1.0);
    const double slow_velocity = moving ?
        p.moving_minimum_velocity + ratio *
            (p.moving_maximum_velocity - p.moving_minimum_velocity) :
        p.static_minimum_velocity + ratio *
            (p.static_maximum_velocity - p.static_minimum_velocity);
    const double relative_velocity = ego_velocity -
                                     obstacle.longitudinal_velocity;
    const double time_to_collision =
        obstacle.front_collision_arc < ego_arc + p.ego_front_offset ? 0.0 :
        (obstacle.front_collision_arc - ego_arc - p.ego_front_offset) /
            std::max(1.0, relative_velocity);
    const double offset = obstacle.longitudinal_velocity *
                          std::max(0.0, time_to_collision - 1.0);
    const double preparation = std::max(
        3.0, slow_velocity * p.target_velocity_time_margin);
    const double deceleration_distance = offset +
        obstacle.front_collision_arc - p.ego_front_offset - ego_arc -
        preparation;
    double start_arc = ego_arc + deceleration_distance;
    double end_arc = start_arc + obstacle.back_collision_arc -
        obstacle.front_collision_arc + preparation;
    if (start_arc > 100.0 || end_arc <= start_arc) continue;
    if (previous) {
      if (previous->start)
        start_arc = lowpass(start_arc,
            dynamicStopArc(path, *previous->start),
            p.distance_filter_gain);
      end_arc = lowpass(end_arc,
          dynamicStopArc(path, previous->end), p.distance_filter_gain);
    }
    double feasible = slow_velocity;
    if (deceleration_distance < 0.0) {
      feasible = previous ? previous->target_velocity :
          std::max(ego_velocity, slow_velocity);
    } else if (ego_velocity >= slow_velocity) {
      double one_shot = slow_velocity;
      if (ego_acceleration < p.minimum_acceleration) {
        const double squared = ego_velocity * ego_velocity +
            2.0 * deceleration_distance * p.minimum_acceleration;
        one_shot = squared < 0.0 ? slow_velocity :
            std::max(std::sqrt(squared), slow_velocity);
      } else {
        one_shot = jerkFeasibleVelocity(
            p.slowdown_jerk, p.slowdown_acceleration,
            ego_acceleration, ego_velocity, deceleration_distance);
      }
      feasible = std::max(slow_velocity, previous ?
          std::min(one_shot, previous->feasible_velocity) : one_shot);
    }
    const double stable = previous ? lowpass(
        feasible, previous->target_velocity, p.velocity_filter_gain) : feasible;
    const auto start = pointAtArc(result.trajectory, std::max(0.0, start_arc));
    const auto end = pointAtArc(result.trajectory, end_arc);
    if (!start || !end) continue;
    MotionVelocityPlan interval;
    interval.slowdowns.push_back({*start, *end, stable});
    result.trajectory = applyMotionVelocityPlans(result.trajectory, {interval});
    result.intervals.slowdowns.push_back(interval.slowdowns.front());
    updated.emplace(obstacle.id, Previous{
        start_arc < 0.0 ? std::nullopt :
            std::optional<MotionVelocityPoint2D>(*start),
        *end, stable, feasible, obstacle.lateral_distance, moving});
  }
  previous_ = std::move(updated);
  return result;
}

}  // planning 네임스페이스
