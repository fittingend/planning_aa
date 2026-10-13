// Copyright 2022-2024 TIER IV, Inc.
// SPDX-License-Identifier: Apache-2.0
// 원본의 직선 전방 투영과 감속 속도 계산입니다.

#include "planning/obstacle_velocity_limiter_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>
#include <utility>

namespace planning {
namespace {
struct LocalPoint { double forward; double lateral; };

LocalPoint toLocal(MotionVelocityPoint2D origin, double heading,
                   MotionVelocityPoint2D target) {
  const double dx = target.x - origin.x;
  const double dy = target.y - origin.y;
  return {std::cos(heading) * dx + std::sin(heading) * dy,
          -std::sin(heading) * dx + std::cos(heading) * dy};
}

std::optional<LocalPoint> clippedSegmentStart(
    LocalPoint a, LocalPoint b, double length, double half_width) {
  const double dx = b.forward - a.forward;
  const double dy = b.lateral - a.lateral;
  double first = 0.0;
  double last = 1.0;
  const auto clip = [&](double p, double q) {
    if (std::abs(p) < 1e-12) return q >= 0.0;
    const double ratio = q / p;
    if (p < 0.0) first = std::max(first, ratio);
    else last = std::min(last, ratio);
    return first <= last;
  };
  if (!clip(-dx, a.forward) || !clip(dx, length - a.forward) ||
      !clip(-dy, a.lateral + half_width) ||
      !clip(dy, half_width - a.lateral)) return std::nullopt;
  const double ratio = dx >= 0.0 ? first : last;
  return LocalPoint{a.forward + ratio * dx,
                    a.lateral + ratio * dy};
}

double yawOf(const TrajectoryPoint &point) {
  return std::atan2(
      2.0 * (point.orientation_w * point.orientation_z +
             point.orientation_x * point.orientation_y),
      1.0 - 2.0 * (point.orientation_y * point.orientation_y +
                   point.orientation_z * point.orientation_z));
}
}  // 익명 네임스페이스

VelocityLimiterSampledPath sampleObstacleVelocityLimiterPath(
    const Trajectory &path, const std::vector<double> &time_from_start,
    const std::size_t ego_index,
    const ObstacleVelocityLimiterParameters &p) {
  if (path.size() != time_from_start.size() || ego_index >= path.size() ||
      p.start_distance < 0.0 ||
      p.maximum_path_length < 0.0 || p.maximum_path_duration < 0.0)
    throw std::invalid_argument("장애물 속도 제한 경로 선택 입력이 유효하지 않습니다");
  std::size_t start = ego_index;
  double start_distance = 0.0;
  while (start + 1 < path.size() && start_distance < p.start_distance) {
    start_distance += std::hypot(path[start + 1].x - path[start].x,
                                 path[start + 1].y - path[start].y);
    ++start;
  }
  std::size_t end = start;
  double length = 0.0;
  double duration = 0.0;
  while (end + 1 < path.size() && length < p.maximum_path_length &&
         duration < p.maximum_path_duration) {
    const double segment = std::hypot(path[end + 1].x - path[end].x,
                                      path[end + 1].y - path[end].y);
    length += segment;
    if (path[end].longitudinal_velocity_mps > 0.0F)
      duration += segment / path[end].longitudinal_velocity_mps;
    ++end;
  }
  VelocityLimiterSampledPath sampled;
  if (p.downsample_factor == 0)
    return {path, time_from_start};
  for (std::size_t i = start; i <= end; i += p.downsample_factor) {
    sampled.path.push_back(path[i]);
    sampled.time_from_start.push_back(time_from_start[i]);
  }
  return sampled;
}

void calculateVelocityLimiterSteeringAngles(
    Trajectory &path, const double wheel_base) {
  if (!std::isfinite(wheel_base) || wheel_base <= 0.0)
    throw std::invalid_argument("차량 휠베이스가 유효하지 않습니다");
  if (path.empty()) return;
  double previous_heading = yawOf(path.front());
  for (std::size_t i = 1; i < path.size(); ++i) {
    const auto &previous = path[i - 1];
    auto &point = path[i];
    const double length = std::hypot(point.x - previous.x,
                                     point.y - previous.y);
    const double heading = yawOf(point);
    const double heading_change = heading - previous_heading;
    previous_heading = heading;
    if (previous.longitudinal_velocity_mps <= 0.0F || length <= 0.0) {
      point.front_wheel_angle_rad = 0.0F;
      continue;
    }
    const double dt = length / previous.longitudinal_velocity_mps;
    point.front_wheel_angle_rad = static_cast<float>(std::atan2(
        wheel_base * heading_change,
        point.longitudinal_velocity_mps * dt));
  }
}

std::vector<std::vector<MotionVelocityPoint2D>> bicycleVelocityProjectionLines(
    const MotionVelocityPoint2D origin, const double heading,
    const double velocity, const double steering_angle,
    const BicycleVelocityProjectionParameters &p) {
  if (!std::isfinite(heading) || !std::isfinite(velocity) ||
      !std::isfinite(steering_angle) || p.wheel_base <= 0.0 ||
      p.duration <= 0.0 || p.points_per_projection < 2)
    throw std::invalid_argument("자전거 전방 투영 입력이 유효하지 않습니다");
  std::vector<double> steering;
  if (p.steering_offset == 0.0) steering.push_back(steering_angle);
  else steering = {steering_angle + p.steering_offset,
                   steering_angle,
                   steering_angle - p.steering_offset};
  std::vector<std::vector<MotionVelocityPoint2D>> result;
  for (const double angle : steering) {
    std::vector<MotionVelocityPoint2D> line;
    line.reserve(p.points_per_projection);
    line.push_back(origin);
    const double rotation_rate = velocity * std::tan(angle) / p.wheel_base;
    const double step = p.duration / (p.points_per_projection - 1);
    for (std::size_t i = 1; i < p.points_per_projection; ++i) {
      const double time = i * step;
      const double current_heading = heading + rotation_rate * time;
      const double length = velocity * time + p.extra_length;
      line.push_back({origin.x + length * std::cos(current_heading),
                      origin.y + length * std::sin(current_heading)});
    }
    result.push_back(std::move(line));
  }
  return result;
}

double bicycleProjectionArcDistance(
    const MotionVelocityPoint2D origin, const double heading,
    const MotionVelocityPoint2D target) {
  const double dx = target.x - origin.x;
  const double dy = target.y - origin.y;
  const double chord = std::hypot(dx, dy);
  const double lateral = -std::sin(heading) * dx +
                          std::cos(heading) * dy;
  if (std::abs(lateral) < 1e-9) return chord;
  const double radius = chord * chord / (2.0 * std::abs(lateral));
  const double angle = 2.0 * std::asin(std::clamp(
      chord / (2.0 * radius), 0.0, 1.0));
  return radius * angle;
}

MotionVelocityPlan planObstacleVelocityLimitParticle(
    const Trajectory &path, const std::vector<double> &time_from_start,
    const std::vector<MotionVelocityPoint2D> &obstacle_points,
    const std::vector<VelocityLimiterSegment> &obstacle_segments,
    const double current_ego_velocity,
    const ObstacleVelocityLimiterParameters &p) {
  if (path.size() != time_from_start.size() ||
      !std::isfinite(current_ego_velocity) ||
      p.minimum_time_to_collision <= 0.0 ||
      p.vehicle_lateral_offset < 0.0 || p.maximum_deceleration < 0.0)
    throw std::invalid_argument("장애물 속도 제한 입력이 유효하지 않습니다");
  MotionVelocityPlan result;
  for (std::size_t i = 0; i < path.size(); ++i) {
    const auto &point = path[i];
    if (!std::isfinite(time_from_start[i]) || time_from_start[i] < 0.0)
      throw std::invalid_argument("경로 시각이 유효하지 않습니다");
    const MotionVelocityPoint2D origin{point.x, point.y};
    const double heading = yawOf(point);
    const double projected_length =
        point.longitudinal_velocity_mps * p.minimum_time_to_collision +
        p.extra_projection_length;
    if (projected_length <= 0.0) continue;
    double nearest = std::numeric_limits<double>::infinity();
    for (const auto &obstacle : obstacle_points) {
      const auto local = toLocal(origin, heading, obstacle);
      if (local.forward >= 0.0 && local.forward <= projected_length &&
          std::abs(local.lateral) <= p.vehicle_lateral_offset)
        nearest = std::min(nearest,
            p.distance_method == VelocityLimiterDistanceMethod::Exact
                ? local.forward
                : std::hypot(local.forward, local.lateral));
    }
    for (const auto &obstacle : obstacle_segments) {
      const auto from = toLocal(origin, heading, obstacle.from);
      const auto to = toLocal(origin, heading, obstacle.to);
      const auto collision = clippedSegmentStart(
          from, to, projected_length, p.vehicle_lateral_offset);
      if (collision) {
        // 근사 거리에서는 접촉 선분의 투영 교점까지의 직선 거리를 사용합니다.
        nearest = std::min(nearest,
            p.distance_method == VelocityLimiterDistanceMethod::Exact
                ? collision->forward
                : std::hypot(collision->forward, collision->lateral));
      }
    }
    if (!std::isfinite(nearest)) continue;
    const double feasible_velocity = current_ego_velocity -
        p.maximum_deceleration * time_from_start[i];
    const double safe_velocity = std::max(
        (nearest - p.extra_projection_length) /
            p.minimum_time_to_collision,
        p.minimum_adjusted_velocity);
    result.slowdowns.push_back({origin, origin,
                                std::max(feasible_velocity, safe_velocity)});
  }
  return result;
}

}  // planning 네임스페이스
