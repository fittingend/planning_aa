// Copyright 2023-2024 TIER IV, Inc.
// SPDX-License-Identifier: Apache-2.0
// 원본 동적 장애물 정지 모듈의 필터와 객체별 시간 상태입니다.

#include "planning/dynamic_obstacle_stop_core.hpp"
#include "planning/crosswalk_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace planning {
namespace {
constexpr double kPi = 3.141592653589793;

double yawOf(const TrajectoryPoint &point) {
  return std::atan2(
      2.0 * (point.orientation_w * point.orientation_z +
             point.orientation_x * point.orientation_y),
      1.0 - 2.0 * (point.orientation_y * point.orientation_y +
                   point.orientation_z * point.orientation_z));
}

std::size_t nearestSegment(const Trajectory &path, MotionVelocityPoint2D point) {
  std::size_t best_index = 0;
  double best_distance = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    const double dx = path[i + 1].x - path[i].x;
    const double dy = path[i + 1].y - path[i].y;
    const double norm = dx * dx + dy * dy;
    const double ratio = norm < 1e-12 ? 0.0 : std::clamp(
        ((point.x - path[i].x) * dx + (point.y - path[i].y) * dy) / norm,
        0.0, 1.0);
    const double distance = std::hypot(
        path[i].x + ratio * dx - point.x, path[i].y + ratio * dy - point.y);
    if (distance < best_distance) {
      best_distance = distance;
      best_index = i;
    }
  }
  return best_index;
}

bool vehicleType(DynamicObjectType type) {
  return type == DynamicObjectType::Car || type == DynamicObjectType::Bus ||
         type == DynamicObjectType::Truck || type == DynamicObjectType::Trailer ||
         type == DynamicObjectType::Motorcycle || type == DynamicObjectType::Bicycle;
}

double lateralDeviationFromPose(double x, double y, double yaw,
                                MotionVelocityPoint2D target) {
  return -(target.x - x) * std::sin(yaw) +
          (target.y - y) * std::cos(yaw);
}

bool unavoidable(const DynamicObject &object, const TrajectoryPoint &ego,
                 const std::optional<TrajectoryPoint> &earliest_stop,
                 const DynamicObstacleStopParameters &p) {
  const double yaw_difference = std::abs(std::remainder(
      object.yaw - yawOf(ego), 2.0 * kPi));
  const double limit = p.ego_lateral_offset + object.width / 2.0 + p.hysteresis;
  bool collision = yaw_difference > 3.0 * kPi / 4.0 &&
      std::abs(lateralDeviationFromPose(
          object.x, object.y, object.yaw, {ego.x, ego.y})) <= limit;
  if (earliest_stop) {
    const double direction = std::atan2(
        object.y - earliest_stop->y, object.x - earliest_stop->x);
    const double direction_difference = std::abs(std::remainder(
        object.yaw - direction, 2.0 * kPi));
    collision |= direction_difference > 3.0 * kPi / 4.0 &&
        std::abs(lateralDeviationFromPose(object.x, object.y, object.yaw,
            {earliest_stop->x, earliest_stop->y})) <= limit;
  }
  return collision;
}

DynamicStopRectangle egoFootprint(
    const TrajectoryPoint &point, const DynamicObstacleStopParameters &p) {
  const double c = std::cos(yawOf(point)), s = std::sin(yawOf(point));
  const auto transform = [&](double x, double y) {
    return MotionVelocityPoint2D{point.x + c * x - s * y,
                                 point.y + s * x + c * y};
  };
  return {transform(p.ego_longitudinal_offset, p.ego_lateral_offset),
          transform(p.ego_longitudinal_offset, -p.ego_lateral_offset),
          transform(0.0, -p.ego_lateral_offset),
          transform(0.0, p.ego_lateral_offset)};
}

std::optional<MotionVelocityPoint2D> edgeIntersection(
    MotionVelocityPoint2D a, MotionVelocityPoint2D b,
    MotionVelocityPoint2D c, MotionVelocityPoint2D d) {
  const double rx = b.x - a.x, ry = b.y - a.y;
  const double sx = d.x - c.x, sy = d.y - c.y;
  const double denominator = rx * sy - ry * sx;
  const double qx = c.x - a.x, qy = c.y - a.y;
  if (std::abs(denominator) < 1e-12) {
    if (std::abs(qx * ry - qy * rx) >= 1e-12)
      return std::nullopt;
    const double length_squared = rx * rx + ry * ry;
    if (length_squared < 1e-12) return std::nullopt;
    const double t0 = (qx * rx + qy * ry) / length_squared;
    const double t1 = ((d.x - a.x) * rx + (d.y - a.y) * ry) /
                      length_squared;
    const double first = std::max(0.0, std::min(t0, t1));
    const double last = std::min(1.0, std::max(t0, t1));
    if (first > last) return std::nullopt;
    return MotionVelocityPoint2D{a.x + first * rx, a.y + first * ry};
  }
  const double t = (qx * sy - qy * sx) / denominator;
  const double u = (qx * ry - qy * rx) / denominator;
  if (t < 0.0 || t > 1.0 || u < 0.0 || u > 1.0)
    return std::nullopt;
  return MotionVelocityPoint2D{a.x + t * rx, a.y + t * ry};
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
}  // 익명 네임스페이스

double dynamicStopMinimumBrakingDistance(
    const double velocity_mps, const double acceleration_mps2,
    const double minimum_acceleration_mps2,
    const double maximum_jerk_mps3, const double minimum_jerk_mps3) {
  if (!std::isfinite(velocity_mps) || velocity_mps < 0.0)
    throw std::invalid_argument("동적 장애물 정지의 차량 속도가 유효하지 않습니다");
  const auto distance = calcCrosswalkDecelDistance(
      velocity_mps, 0.0, acceleration_mps2,
      minimum_acceleration_mps2, maximum_jerk_mps3,
      minimum_jerk_mps3);
  return std::max(0.0, distance.value_or(0.0));
}

double dynamicStopArc(const Trajectory &path, MotionVelocityPoint2D point) {
  if (path.size() < 2) throw std::invalid_argument("동적 장애물 경로가 짧습니다");
  const std::size_t segment = nearestSegment(path, point);
  double arc = 0.0;
  for (std::size_t i = 0; i < segment; ++i)
    arc += std::hypot(path[i + 1].x - path[i].x, path[i + 1].y - path[i].y);
  const double dx = path[segment + 1].x - path[segment].x;
  const double dy = path[segment + 1].y - path[segment].y;
  const double length = std::hypot(dx, dy);
  if (length > 1e-9)
    arc += ((point.x - path[segment].x) * dx +
            (point.y - path[segment].y) * dy) / length;
  return arc;
}

double dynamicStopLateralOffset(const Trajectory &path,
                                MotionVelocityPoint2D point) {
  if (path.size() < 2) throw std::invalid_argument("동적 장애물 경로가 짧습니다");
  const std::size_t i = nearestSegment(path, point);
  const double dx = path[i + 1].x - path[i].x;
  const double dy = path[i + 1].y - path[i].y;
  const double length = std::hypot(dx, dy);
  return length < 1e-9 ? 0.0 :
      (dx * (point.y - path[i].y) - dy * (point.x - path[i].x)) / length;
}

std::vector<DynamicObject> filterDynamicStopObjects(
    const std::vector<DynamicObject> &objects, const Trajectory &path,
    const TrajectoryPoint &ego, const double first_offset,
    const std::optional<TrajectoryPoint> earliest_stop,
    const DynamicObstacleStopParameters &p, const double hysteresis) {
  if (path.size() < 2 || !std::isfinite(first_offset) ||
      !std::isfinite(hysteresis)) {
    throw std::invalid_argument("동적 장애물 필터 입력이 유효하지 않습니다");
  }
  std::vector<DynamicObject> filtered;
  const double ego_arc = dynamicStopArc(path, {ego.x, ego.y});
  for (const auto &object : objects) {
    if (!vehicleType(object.type) || object.velocity < p.minimum_object_velocity)
      continue;
    const MotionVelocityPoint2D position{object.x, object.y};
    const double lateral = std::abs(dynamicStopLateralOffset(path, position));
    if (lateral > p.minimum_object_distance_from_ego_trajectory +
                      p.ego_lateral_offset + object.width / 2.0 + hysteresis)
      continue;
    const double longitude = dynamicStopArc(path, position) - ego_arc;
    if (std::abs(longitude) <= first_offset +
                               p.ego_longitudinal_offset + object.length / 2.0)
      continue;
    if (p.ignore_unavoidable_collisions &&
        unavoidable(object, ego, earliest_stop, p)) continue;
    filtered.push_back(object);
  }
  return filtered;
}

DynamicStopRectangle dynamicStopForwardFootprint(
    const DynamicObject &object,
    const DynamicObstacleStopParameters &p, const double hysteresis) {
  const double front = object.length / 2.0 + object.velocity * p.time_horizon;
  const double back = object.length / 2.0;
  const double half_width = (object.width + p.extra_object_width) / 2.0 +
                            hysteresis;
  const double c = std::cos(object.yaw), s = std::sin(object.yaw);
  const auto rotate = [&](double x, double y) {
    return MotionVelocityPoint2D{
        object.x + c * x - s * y, object.y + s * x + c * y};
  };
  return {rotate(front, -half_width), rotate(front, half_width),
          rotate(back, half_width), rotate(back, -half_width)};
}

std::vector<DynamicStopCollision> findDynamicStopCollisions(
    const Trajectory &path, const MotionVelocityPoint2D ego,
    const std::vector<DynamicObject> &objects,
    const DynamicObstacleStopParameters &p, const double hysteresis) {
  if (path.size() < 2) throw std::invalid_argument("동적 장애물 충돌 경로가 짧습니다");
  std::vector<DynamicStopCollision> collisions;
  const double ego_arc = dynamicStopArc(path, ego);
  for (const auto &object : objects) {
    const auto obstacle = dynamicStopForwardFootprint(object, p, hysteresis);
    double closest_arc = std::numeric_limits<double>::infinity();
    std::optional<MotionVelocityPoint2D> closest_point;
    for (const auto &point : path) {
      if (std::abs(std::remainder(
              yawOf(point) - object.yaw, 2.0 * kPi)) <= 3.0 * kPi / 4.0)
        continue;
      const auto footprint = egoFootprint(point, p);
      for (std::size_t a = 0; a < footprint.size(); ++a) {
        for (std::size_t b = 0; b < obstacle.size(); ++b) {
          const auto intersection = edgeIntersection(
              footprint[a], footprint[(a + 1) % footprint.size()],
              obstacle[b], obstacle[(b + 1) % obstacle.size()]);
          if (!intersection) continue;
          const double arc = dynamicStopArc(path, *intersection) - ego_arc;
          if (arc < closest_arc) {
            closest_arc = arc;
            closest_point = intersection;
          }
        }
      }
    }
    if (closest_point) collisions.push_back({object.id, *closest_point});
  }
  return collisions;
}

std::optional<MotionVelocityPoint2D> dynamicStopPointFromCollision(
    const Trajectory &path, const MotionVelocityPoint2D ego,
    const MotionVelocityPoint2D collision,
    const double earliest_stop_distance,
    const DynamicObstacleStopParameters &p) {
  if (path.size() < 2 || !std::isfinite(earliest_stop_distance) ||
      earliest_stop_distance < 0.0)
    throw std::invalid_argument("동적 장애물 정지 거리 입력이 유효하지 않습니다");
  const double ego_arc = dynamicStopArc(path, ego);
  const double collision_arc = dynamicStopArc(path, collision);
  const double arc_difference = ego_arc - collision_arc;
  const bool can_stop_before_limit = arc_difference <
      earliest_stop_distance - p.ego_longitudinal_offset - p.stop_distance_buffer;
  return pointAtArc(path, can_stop_before_limit ?
      collision_arc - p.stop_distance_buffer - p.ego_longitudinal_offset :
      ego_arc + earliest_stop_distance);
}

DynamicObstacleStopDecisionCore::DynamicObstacleStopDecisionCore(
    DynamicObstacleStopParameters parameters) : parameters_(parameters) {}

void DynamicObstacleStopDecisionCore::reset() { states_.clear(); }

void DynamicObstacleStopDecisionCore::update(
    const Trajectory &path,
    const std::vector<DynamicStopCollision> &collisions,
    const double now) {
  if (path.size() < 2 || !std::isfinite(now))
    throw std::invalid_argument("동적 장애물 시간 상태 입력이 유효하지 않습니다");
  for (auto &[id, state] : states_) state.detected = false;
  for (const auto &collision : collisions) {
    const auto found = states_.find(collision.object_id);
    if (found == states_.end()) {
      states_[collision.object_id].collision = collision.point;
    } else {
      found->second.detected = true;
      if (dynamicStopArc(path, collision.point) <
          dynamicStopArc(path, found->second.collision))
        found->second.collision = collision.point;
    }
  }
  for (auto it = states_.begin(); it != states_.end();) {
    auto &state = it->second;
    if (state.detected) {
      if (!state.first_detection_time) state.first_detection_time = now;
      if (now - *state.first_detection_time >= parameters_.add_duration_buffer)
        state.last_stop_decision_time = now;
    } else if (!state.last_stop_decision_time ||
               now - *state.last_stop_decision_time >=
                   parameters_.remove_duration_buffer) {
      state.first_detection_time.reset();
    }
    if (!state.first_detection_time) it = states_.erase(it);
    else ++it;
  }
}

std::optional<MotionVelocityPoint2D>
DynamicObstacleStopDecisionCore::earliestCollision(
    const Trajectory &path, const MotionVelocityPoint2D ego) const {
  std::optional<MotionVelocityPoint2D> earliest;
  double distance = std::numeric_limits<double>::infinity();
  const double ego_arc = dynamicStopArc(path, ego);
  for (const auto &[id, state] : states_) {
    if (!state.first_detection_time || !state.last_stop_decision_time) continue;
    const double current = dynamicStopArc(path, state.collision) - ego_arc;
    if (current < distance) { distance = current; earliest = state.collision; }
  }
  return earliest;
}

bool DynamicObstacleStopDecisionCore::hasActiveDecision() const {
  for (const auto &[id, state] : states_)
    if (state.first_detection_time && state.last_stop_decision_time) return true;
  return false;
}

DynamicObstacleStopCore::DynamicObstacleStopCore(
    DynamicObstacleStopParameters parameters)
    : parameters_(parameters), decisions_(parameters) {}

void DynamicObstacleStopCore::reset() { decisions_.reset(); }

MotionVelocityPlan DynamicObstacleStopCore::plan(
    const Trajectory &path, const TrajectoryPoint &ego,
    const std::vector<DynamicObject> &objects,
    const double earliest_stop_distance,
    const double longitudinal_offset_to_first_point,
    const double now_seconds) {
  MotionVelocityPlan result;
  if (path.size() < 2) return result;
  if (!std::isfinite(earliest_stop_distance) || earliest_stop_distance < 0.0)
    throw std::invalid_argument("동적 장애물 최소 정지 거리가 유효하지 않습니다");
  const MotionVelocityPoint2D ego_point{ego.x, ego.y};
  const auto earliest_stop_point = pointAtArc(
      path, dynamicStopArc(path, ego_point) + earliest_stop_distance);
  std::optional<TrajectoryPoint> earliest_stop_pose;
  if (earliest_stop_point) {
    TrajectoryPoint pose;
    pose.x = earliest_stop_point->x;
    pose.y = earliest_stop_point->y;
    earliest_stop_pose = pose;
  }
  const double hysteresis = decisions_.hasActiveDecision() ?
      parameters_.hysteresis : 0.0;
  const auto filtered = filterDynamicStopObjects(
      objects, path, ego, longitudinal_offset_to_first_point,
      earliest_stop_pose, parameters_, hysteresis);
  const auto collisions = findDynamicStopCollisions(
      path, ego_point, filtered, parameters_, hysteresis);
  decisions_.update(path, collisions, now_seconds);
  const auto collision = decisions_.earliestCollision(path, ego_point);
  if (collision) {
    const auto stop = dynamicStopPointFromCollision(
        path, ego_point, *collision, earliest_stop_distance, parameters_);
    if (stop) result.stop_points.push_back(*stop);
  }
  return result;
}

}  // planning 네임스페이스
