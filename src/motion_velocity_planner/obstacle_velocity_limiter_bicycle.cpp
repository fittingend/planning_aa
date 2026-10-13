// Copyright 2022-2024 TIER IV, Inc.
// SPDX-License-Identifier: Apache-2.0
// 자전거 전방 투영 영역과 정적 장애물의 교차를 계산합니다.

#include "planning/obstacle_velocity_limiter_core.hpp"

#include <boost/geometry.hpp>
#include <boost/geometry/strategies/buffer.hpp>
#include <boost/geometry/strategies/cartesian/buffer_end_flat.hpp>
#include <boost/geometry/strategies/cartesian/buffer_join_miter.hpp>
#include <boost/geometry/strategies/cartesian/buffer_point_circle.hpp>
#include <boost/geometry/strategies/cartesian/buffer_side_straight.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace planning {
namespace {
namespace bg = boost::geometry;
using Point = bg::model::d2::point_xy<double>;
using Line = bg::model::linestring<Point>;
using Polygon = bg::model::polygon<Point>;

Point perpendicularPoint(MotionVelocityPoint2D a,
                         MotionVelocityPoint2D b, double offset) {
  const double dx = b.x - a.x;
  const double dy = b.y - a.y;
  const double length = std::hypot(dx, dy);
  if (length < 1e-9) return {b.x, b.y};
  return {b.x - offset * dy / length,
          b.y + offset * dx / length};
}

Polygon footprint(
    const std::vector<std::vector<MotionVelocityPoint2D>> &lines,
    double lateral_offset) {
  Polygon result;
  if (lines.size() == 1) {
    Line line;
    for (const auto &point : lines.front())
      line.emplace_back(point.x, point.y);
    bg::model::multi_polygon<Polygon> buffered;
    namespace strategy = bg::strategy::buffer;
    bg::buffer(line, buffered,
               strategy::distance_symmetric<double>(lateral_offset),
               strategy::side_straight(), strategy::join_miter(),
               strategy::end_flat(), strategy::point_square());
    if (!buffered.empty()) result = buffered.front();
  } else if (lines.size() >= 3) {
    const auto &left = lines[0];
    const auto &middle = lines[1];
    const auto &right = lines[2];
    if (left.size() < 2 || middle.size() < 2 || right.size() < 2)
      return result;
    result.outer().push_back(perpendicularPoint(
        left[1], left[0], -lateral_offset));
    for (std::size_t i = 0; i + 1 < left.size(); ++i)
      result.outer().push_back(perpendicularPoint(
          left[i], left[i + 1], lateral_offset));
    result.outer().push_back(perpendicularPoint(
        middle[middle.size() - 2], middle.back(), lateral_offset));
    result.outer().push_back(perpendicularPoint(
        middle[middle.size() - 2], middle.back(), -lateral_offset));
    result.outer().push_back(perpendicularPoint(
        right[right.size() - 2], right.back(), -lateral_offset));
    for (std::size_t i = right.size() - 1; i > 0; --i)
      result.outer().push_back(perpendicularPoint(
          right[i], right[i - 1], lateral_offset));
    result.outer().push_back(result.outer().front());
    bg::correct(result);
  }
  return result;
}

double yawOf(const TrajectoryPoint &point) {
  return std::atan2(
      2.0 * (point.orientation_w * point.orientation_z +
             point.orientation_x * point.orientation_y),
      1.0 - 2.0 * (point.orientation_y * point.orientation_y +
                   point.orientation_z * point.orientation_z));
}
}  // 익명 네임스페이스

MotionVelocityPlan planObstacleVelocityLimitBicycle(
    const Trajectory &path, const std::vector<double> &time_from_start,
    const std::vector<MotionVelocityPoint2D> &obstacle_points,
    const std::vector<VelocityLimiterSegment> &obstacle_segments,
    const double current_ego_velocity,
    const ObstacleVelocityLimiterParameters &velocity,
    const BicycleVelocityProjectionParameters &projection) {
  if (path.size() != time_from_start.size() ||
      !std::isfinite(current_ego_velocity) ||
      velocity.minimum_time_to_collision <= 0.0 ||
      velocity.vehicle_lateral_offset < 0.0)
    throw std::invalid_argument("자전거 속도 제한 입력이 유효하지 않습니다");
  MotionVelocityPlan plan;
  for (std::size_t i = 0; i < path.size(); ++i) {
    const auto &point = path[i];
    if (!std::isfinite(time_from_start[i]) || time_from_start[i] < 0.0)
      throw std::invalid_argument("자전거 경로 시각이 유효하지 않습니다");
    const MotionVelocityPoint2D origin{point.x, point.y};
    const double heading = yawOf(point);
    auto settings = projection;
    settings.duration = velocity.minimum_time_to_collision;
    settings.extra_length = velocity.extra_projection_length;
    const auto lines = bicycleVelocityProjectionLines(
        origin, heading, point.longitudinal_velocity_mps,
        point.front_wheel_angle_rad, settings);
    const auto shape = footprint(lines, velocity.vehicle_lateral_offset);
    if (bg::is_empty(shape)) continue;
    double nearest = std::numeric_limits<double>::infinity();
    const auto consider = [&](const Point &p) {
      nearest = std::min(nearest,
          velocity.distance_method == VelocityLimiterDistanceMethod::Exact
              ? bicycleProjectionArcDistance(
                    origin, heading, {p.x(), p.y()})
              : std::hypot(p.x() - origin.x, p.y() - origin.y));
    };
    for (const auto &obstacle : obstacle_points) {
      const Point candidate(obstacle.x, obstacle.y);
      if (bg::covered_by(candidate, shape)) consider(candidate);
    }
    for (const auto &obstacle : obstacle_segments) {
      const Line segment{{obstacle.from.x, obstacle.from.y},
                         {obstacle.to.x, obstacle.to.y}};
      std::vector<Point> intersections;
      bg::intersection(segment, shape, intersections);
      if (intersections.empty() && bg::within(segment, shape)) {
        consider(segment.front());
        consider(segment.back());
      }
      for (const auto &candidate : intersections) consider(candidate);
    }
    if (!std::isfinite(nearest)) continue;
    const double feasible = current_ego_velocity -
        velocity.maximum_deceleration * time_from_start[i];
    const double safe = std::max(
        (nearest - velocity.extra_projection_length) /
            velocity.minimum_time_to_collision,
        velocity.minimum_adjusted_velocity);
    plan.slowdowns.push_back({origin, origin, std::max(feasible, safe)});
  }
  return plan;
}

}  // planning 네임스페이스
