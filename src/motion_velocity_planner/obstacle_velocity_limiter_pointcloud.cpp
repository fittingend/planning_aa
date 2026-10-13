// Copyright 2022-2024 TIER IV, Inc.
// SPDX-License-Identifier: Apache-2.0
// 점군에 적용하는 경로 주변·이동 객체 제외 다각형입니다.

#include "planning/obstacle_velocity_limiter_core.hpp"

#include <boost/geometry.hpp>
#include <boost/geometry/strategies/buffer.hpp>
#include <boost/geometry/strategies/cartesian/buffer_end_flat.hpp>
#include <boost/geometry/strategies/cartesian/buffer_join_miter.hpp>
#include <boost/geometry/strategies/cartesian/buffer_point_circle.hpp>
#include <boost/geometry/strategies/cartesian/buffer_side_straight.hpp>

#include <cmath>
#include <stdexcept>

namespace planning {
namespace {
namespace bg = boost::geometry;
using Point = bg::model::d2::point_xy<double>;
using Polygon = bg::model::polygon<Point>;
using Line = bg::model::linestring<Point>;

Polygon makePolygon(const VelocityLimiterPolygon &vertices) {
  Polygon polygon;
  for (const auto &vertex : vertices)
    polygon.outer().emplace_back(vertex.x, vertex.y);
  if (vertices.size() >= 3) bg::correct(polygon);
  return polygon;
}

VelocityLimiterPolygon verticesOf(const Polygon &polygon) {
  VelocityLimiterPolygon vertices;
  for (const auto &vertex : polygon.outer())
    vertices.push_back({vertex.x(), vertex.y()});
  return vertices;
}
}  // 익명 네임스페이스

std::vector<MotionVelocityPoint2D> filterVelocityLimiterPointCloud(
    const std::vector<MotionVelocityPoint2D> &points,
    const VelocityLimiterObstacleMasks &masks) {
  if (!masks.positive.empty() && masks.positive.size() < 3)
    throw std::invalid_argument("점군 포함 영역의 꼭짓점이 부족합니다");
  const auto positive = makePolygon(masks.positive);
  std::vector<Polygon> negative;
  negative.reserve(masks.negative.size());
  for (const auto &mask : masks.negative) {
    if (mask.empty()) continue;
    if (mask.size() < 3)
      throw std::invalid_argument("점군 제외 영역의 꼭짓점이 부족합니다");
    negative.push_back(makePolygon(mask));
  }
  std::vector<MotionVelocityPoint2D> filtered;
  filtered.reserve(points.size());
  for (const auto &point : points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y)) continue;
    const Point candidate(point.x, point.y);
    if (!masks.positive.empty() && !bg::covered_by(candidate, positive))
      continue;
    bool excluded = false;
    for (const auto &mask : negative) {
      if (bg::covered_by(candidate, mask)) {
        excluded = true;
        break;
      }
    }
    if (!excluded) filtered.push_back(point);
  }
  return filtered;
}

VelocityLimiterPolygon velocityLimiterObjectPolygon(
    const VelocityLimiterObject &object, const double buffer) {
  if (!std::isfinite(object.heading) || !std::isfinite(object.length) ||
      !std::isfinite(object.width) || !std::isfinite(buffer) ||
      !std::isfinite(object.center.x) || !std::isfinite(object.center.y) ||
      object.length + buffer <= 0.0 || object.width + buffer <= 0.0)
    throw std::invalid_argument("장애물 객체 외곽 입력이 유효하지 않습니다");
  const double longitudinal = (object.length + buffer) / 2.0;
  const double lateral = (object.width + buffer) / 2.0;
  const double c = std::cos(object.heading);
  const double s = std::sin(object.heading);
  const MotionVelocityPoint2D corners[] = {
      {longitudinal, lateral}, {-longitudinal, lateral},
      {-longitudinal, -lateral}, {longitudinal, -lateral}};
  VelocityLimiterPolygon result;
  for (const auto &corner : corners)
    result.push_back({object.center.x + c * corner.x - s * corner.y,
                      object.center.y + s * corner.x + c * corner.y});
  result.push_back(result.front());
  return result;
}

std::vector<VelocityLimiterPolygon> velocityLimiterMovingObjectMasks(
    const std::vector<VelocityLimiterObject> &objects,
    const double buffer, const double minimum_velocity) {
  if (!std::isfinite(minimum_velocity) || minimum_velocity < 0.0)
    throw std::invalid_argument("장애물 객체 속도 경계가 유효하지 않습니다");
  std::vector<VelocityLimiterPolygon> masks;
  for (const auto &object : objects) {
    if (std::hypot(object.velocity_x, object.velocity_y) >= minimum_velocity)
      masks.push_back(velocityLimiterObjectPolygon(object, buffer));
  }
  return masks;
}

VelocityLimiterPolygon velocityLimiterTrajectoryMask(
    const Trajectory &path, const double lateral_offset) {
  if (!std::isfinite(lateral_offset) || lateral_offset < 0.0)
    throw std::invalid_argument("경로 제외 영역 폭이 유효하지 않습니다");
  if (path.size() < 2 || lateral_offset == 0.0) return {};
  Line line;
  for (const auto &point : path)
    line.emplace_back(point.x, point.y);
  bg::model::multi_polygon<Polygon> buffered;
  namespace strategy = bg::strategy::buffer;
  bg::buffer(line, buffered,
             strategy::distance_symmetric<double>(lateral_offset),
             strategy::side_straight(), strategy::join_miter(),
             strategy::end_flat(), strategy::point_square());
  if (buffered.empty()) return {};
  return verticesOf(buffered.front());
}

MotionVelocityPlan planObstacleVelocityLimitPointCloud(
    const Trajectory &path, const std::vector<double> &time_from_start,
    const std::size_t ego_index, const double current_ego_velocity,
    const std::vector<MotionVelocityPoint2D> &pointcloud,
    const VelocityLimiterObstacleMasks &masks,
    const ObstacleVelocityLimiterParameters &parameters,
    const BicycleVelocityProjectionParameters &bicycle_parameters,
    const bool apply_masks) {
  auto prepared = path;
  if (parameters.calculate_steering_angles)
    calculateVelocityLimiterSteeringAngles(
        prepared, bicycle_parameters.wheel_base);
  const auto sampled = sampleObstacleVelocityLimiterPath(
      prepared, time_from_start, ego_index, parameters);
  // 원본의 makeShared()는 복사본이므로 기본 계획 경로에는 필터 결과가 반영되지 않습니다.
  const auto points = apply_masks
      ? filterVelocityLimiterPointCloud(pointcloud, masks) : pointcloud;
  if (parameters.model == VelocityLimiterProjectionModel::Bicycle)
    return planObstacleVelocityLimitBicycle(
        sampled.path, sampled.time_from_start, points, {},
        current_ego_velocity, parameters, bicycle_parameters);
  return planObstacleVelocityLimitParticle(
      sampled.path, sampled.time_from_start, points, {},
      current_ego_velocity, parameters);
}

}  // planning 네임스페이스
