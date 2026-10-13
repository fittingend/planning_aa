// Copyright 2020 Tier IV, Inc.
// SPDX-License-Identifier: Apache-2.0
// 원본 횡단보도 모듈의 경계 교차와 기본 정지 위치를 계산합니다.

#include "planning/crosswalk_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace planning {
namespace {

double cross(CrosswalkPoint a, CrosswalkPoint b) {
  return a.x * b.y - a.y * b.x;
}

CrosswalkPoint minus(CrosswalkPoint a, CrosswalkPoint b) {
  return {a.x - b.x, a.y - b.y};
}

std::vector<double> pathArcs(const Trajectory &path) {
  std::vector<double> arc(path.size(), 0.0);
  for (std::size_t i = 1; i < path.size(); ++i) {
    const double step = std::hypot(path[i].x - path[i - 1].x,
                                   path[i].y - path[i - 1].y);
    if (step <= 1.0e-9) throw std::invalid_argument("경로에 중복 좌표가 있습니다");
    arc[i] = arc[i - 1] + step;
  }
  return arc;
}

CrosswalkPoint pointAtArc(
    const Trajectory &path, const std::vector<double> &arc, double query) {
  const auto upper = std::upper_bound(arc.begin(), arc.end(), query);
  const std::size_t next = std::min<std::size_t>(
      std::distance(arc.begin(), upper), path.size() - 1);
  const std::size_t prev = next - 1;
  const double t = (query - arc[prev]) / (arc[next] - arc[prev]);
  return {path[prev].x + t * (path[next].x - path[prev].x),
          path[prev].y + t * (path[next].y - path[prev].y)};
}

double nearestArc(const Trajectory &path, const std::vector<double> &arc,
                  CrosswalkPoint point) {
  double best_distance = std::numeric_limits<double>::infinity();
  double best_arc = 0.0;
  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    const double dx = path[i + 1].x - path[i].x;
    const double dy = path[i + 1].y - path[i].y;
    const double length2 = dx * dx + dy * dy;
    if (length2 < 1.0e-12) continue;
    const double ratio = std::clamp(
        ((point.x - path[i].x) * dx + (point.y - path[i].y) * dy) / length2,
        0.0, 1.0);
    const double distance2 = std::pow(point.x - path[i].x - ratio * dx, 2) +
                             std::pow(point.y - path[i].y - ratio * dy, 2);
    if (distance2 < best_distance) {
      best_distance = distance2;
      best_arc = arc[i] + ratio * std::sqrt(length2);
    }
  }
  return best_arc;
}

double nearestLateralDistance(const Trajectory &path, CrosswalkPoint point) {
  double best_distance = std::numeric_limits<double>::infinity();
  double selected = 0.0;
  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    const double dx = path[i + 1].x - path[i].x;
    const double dy = path[i + 1].y - path[i].y;
    const double length2 = dx * dx + dy * dy;
    if (length2 < 1.0e-12) continue;
    const double ratio = std::clamp(
        ((point.x - path[i].x) * dx + (point.y - path[i].y) * dy) / length2,
        0.0, 1.0);
    const double offset_x = point.x - path[i].x - ratio * dx;
    const double offset_y = point.y - path[i].y - ratio * dy;
    const double distance = std::hypot(offset_x, offset_y);
    if (distance < best_distance) {
      best_distance = distance;
      selected = (dx * offset_y - dy * offset_x) >= 0.0
          ? distance : -distance;
    }
  }
  return selected;
}

void insertStop(CrosswalkResult &result, const Trajectory &path,
                const std::vector<double> &arc, double stop) {
  result.stop_arc_m = stop;
  result.stop_point = pointAtArc(path, arc, stop);
  result.trajectory = path;
  const auto upper = std::upper_bound(arc.begin(), arc.end(), stop);
  const std::size_t next = std::distance(arc.begin(), upper);
  if (next >= path.size()) return;
  const double distance_to_next = std::hypot(
      path[next].x - result.stop_point->x,
      path[next].y - result.stop_point->y);
  const double distance_to_prev = std::hypot(
      path[next - 1].x - result.stop_point->x,
      path[next - 1].y - result.stop_point->y);
  std::size_t first_zero = next;
  if (distance_to_prev <= 0.05) {
    first_zero = next - 1;
  } else if (distance_to_next > 0.05) {
    const double t = (stop - arc[next - 1]) / (arc[next] - arc[next - 1]);
    auto inserted = path[next - 1];
    inserted.x = result.stop_point->x;
    inserted.y = result.stop_point->y;
    inserted.z += t * (path[next].z - inserted.z);
    inserted.longitudinal_velocity_mps = 0.0F;
    result.trajectory.insert(result.trajectory.begin() + next, inserted);
  }
  for (std::size_t i = first_zero; i < result.trajectory.size(); ++i) {
    result.trajectory[i].longitudinal_velocity_mps =
        std::min(result.trajectory[i].longitudinal_velocity_mps, 0.0F);
  }
}

}  // 익명 네임스페이스

CrosswalkResult planCrosswalkDefaultStop(
    const Trajectory &path, const std::vector<CrosswalkPoint> &polygon,
    CrosswalkParameters parameters) {
  if (!std::isfinite(parameters.stop_distance_m) ||
      !std::isfinite(parameters.front_offset_m) ||
      parameters.stop_distance_m < 0.0 || parameters.front_offset_m < 0.0) {
    throw std::invalid_argument("횡단보도 정지 거리 설정이 잘못되었습니다");
  }
  CrosswalkResult result;
  result.trajectory = path;
  if (path.size() < 2 || polygon.size() < 3) return result;
  const auto arc = pathArcs(path);
  std::vector<double> intersections;
  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    const CrosswalkPoint p{path[i].x, path[i].y};
    const CrosswalkPoint r{path[i + 1].x - p.x, path[i + 1].y - p.y};
    for (std::size_t j = 0; j < polygon.size(); ++j) {
      const auto q = polygon[j];
      const auto s = minus(polygon[(j + 1) % polygon.size()], q);
      const double denominator = cross(r, s);
      if (std::abs(denominator) <= 1.0e-12) continue;
      const auto delta = minus(q, p);
      const double t = cross(delta, s) / denominator;
      const double u = cross(delta, r) / denominator;
      if (t >= 0.0 && t <= 1.0 && u >= 0.0 && u <= 1.0) {
        const double crossing = arc[i] + t * (arc[i + 1] - arc[i]);
        if (std::none_of(intersections.begin(), intersections.end(),
            [crossing](double value) { return std::abs(value - crossing) < 1.0e-6; })) {
          intersections.push_back(crossing);
        }
      }
    }
  }
  if (intersections.empty()) return result;
  std::sort(intersections.begin(), intersections.end());
  result.entry_arc_m = intersections.front();
  result.exit_arc_m = intersections.size() > 1 ? intersections[1] : intersections.front();
  const double stop = *result.entry_arc_m - parameters.stop_distance_m -
                      parameters.front_offset_m;
  if (stop < 0.0 || stop > arc.back()) return result;
  insertStop(result, path, arc, stop);
  return result;
}

CrosswalkResult planCrosswalkObjectStop(
    const Trajectory &path, const std::vector<CrosswalkPoint> &polygon,
    CrosswalkPoint ego, const CrosswalkCollisionPoint &collision,
    double weak_braking_distance_m, double strong_braking_distance_m,
    CrosswalkParameters crosswalk_parameters,
    CrosswalkObjectStopParameters object_parameters) {
  if (!std::isfinite(weak_braking_distance_m) ||
      !std::isfinite(strong_braking_distance_m) ||
      !std::isfinite(collision.ego_distance_m) ||
      !std::isfinite(object_parameters.preferred_margin_m) ||
      !std::isfinite(object_parameters.limit_margin_m) ||
      !std::isfinite(object_parameters.overrun_threshold_m) ||
      object_parameters.preferred_margin_m < 0.0 ||
      object_parameters.limit_margin_m < 0.0 ||
      object_parameters.overrun_threshold_m < 0.0) {
    throw std::invalid_argument("횡단보도 객체 정지 거리 설정이 잘못되었습니다");
  }
  auto result = planCrosswalkDefaultStop(path, polygon, crosswalk_parameters);
  if (path.size() < 2 || !result.entry_arc_m) return result;
  const auto arc = pathArcs(path);
  const double ego_arc = nearestArc(path, arc, ego);
  const double preferred = collision.ego_distance_m -
      crosswalk_parameters.front_offset_m - object_parameters.preferred_margin_m;
  const double limit = collision.ego_distance_m -
      crosswalk_parameters.front_offset_m - object_parameters.limit_margin_m;
  const double capped_preferred = result.stop_arc_m
      ? std::min(preferred, *result.stop_arc_m - ego_arc) : preferred;
  const double chosen = weak_braking_distance_m < capped_preferred
      ? capped_preferred
      : weak_braking_distance_m < limit ? weak_braking_distance_m : limit;
  if (chosen < strong_braking_distance_m - object_parameters.overrun_threshold_m ||
      ego_arc + chosen < 0.0 || ego_arc + chosen > arc.back()) {
    result.trajectory = path;
    result.stop_arc_m.reset();
    result.stop_point.reset();
    return result;
  }
  insertStop(result, path, arc, ego_arc + chosen);
  return result;
}

CrosswalkResult applyCrosswalkRestartSuppression(
    const Trajectory &path, CrosswalkResult result,
    CrosswalkPoint ego, bool vehicle_stopped,
    double minimum_distance_m, double maximum_distance_m) {
  if (!std::isfinite(minimum_distance_m) ||
      !std::isfinite(maximum_distance_m) ||
      minimum_distance_m < 0.0 || maximum_distance_m <= minimum_distance_m) {
    throw std::invalid_argument("횡단보도 재출발 억제 거리가 잘못되었습니다");
  }
  if (!vehicle_stopped || !result.stop_arc_m || path.size() < 2) return result;
  const auto arc = pathArcs(path);
  const double ego_arc = nearestArc(path, arc, ego);
  const double distance = *result.stop_arc_m - ego_arc;
  if (minimum_distance_m < distance && distance < maximum_distance_m) {
    insertStop(result, path, arc, ego_arc);
  }
  return result;
}

CrosswalkResult planCrosswalkStuckVehicleStop(
    const Trajectory &path, const std::vector<CrosswalkPoint> &polygon,
    CrosswalkPoint ego, double ego_velocity_mps,
    double ego_acceleration_mps2,
    const std::vector<CrosswalkStuckVehicle> &objects,
    bool road_has_turn_direction, bool road_has_traffic_light,
    CrosswalkParameters crosswalk_parameters,
    CrosswalkStuckParameters stuck_parameters) {
  CrosswalkResult result;
  result.trajectory = path;
  if (!stuck_parameters.enable_in_intersection &&
      road_has_turn_direction && road_has_traffic_light) return result;
  if (path.size() < 2) return result;
  const auto default_stop = planCrosswalkDefaultStop(
      path, polygon, crosswalk_parameters);
  if (!default_stop.stop_arc_m || !default_stop.entry_arc_m ||
      !default_stop.exit_arc_m) return result;
  const auto arc = pathArcs(path);
  const double ego_arc = nearestArc(path, arc, ego);
  for (const auto &object : objects) {
    if (!object.vehicle_type || object.speed_mps > stuck_parameters.maximum_speed_mps ||
        object.length_m <= 0.0 ||
        std::abs(nearestLateralDistance(path, object.position)) >
            stuck_parameters.maximum_lateral_offset_m) continue;
    const double object_arc = nearestArc(path, arc, object.position);
    const double rear_arc = object_arc - object.length_m / 2.0;
    if (rear_arc <= *default_stop.entry_arc_m ||
        rear_arc - *default_stop.exit_arc_m >=
            stuck_parameters.ego_vehicle_length_m +
            stuck_parameters.required_clearance_m) continue;
    const auto braking = calcCrosswalkDecelDistance(
        ego_velocity_mps, 0.0, ego_acceleration_mps2,
        stuck_parameters.minimum_acceleration_mps2,
        stuck_parameters.recovery_jerk_mps3,
        stuck_parameters.braking_jerk_mps3);
    if (!braking) return result;
    const double stop = ego_arc + std::max(
        *braking, *default_stop.stop_arc_m - ego_arc);
    if (stop < 0.0 || stop > arc.back()) return result;
    result.entry_arc_m = default_stop.entry_arc_m;
    result.exit_arc_m = default_stop.exit_arc_m;
    insertStop(result, path, arc, stop);
    return result;
  }
  return result;
}

CrosswalkCollisionState classifyCrosswalkPass(
    double time_to_collision_s, double time_to_vehicle_s,
    CrosswalkCollisionState previous, CrosswalkPassParameters parameters) {
  if (!std::isfinite(time_to_collision_s) || !std::isfinite(time_to_vehicle_s) ||
      time_to_collision_s < 0.0 || time_to_vehicle_s < 0.0) {
    throw std::invalid_argument("충돌과 차량 도착 시각이 잘못되었습니다");
  }
  const double first_extra = previous == CrosswalkCollisionState::EgoPassFirst
                                 ? 0.0 : parameters.ego_pass_first_additional_margin_s;
  if (time_to_collision_s + parameters.ego_pass_first_margin_s + first_extra <
      time_to_vehicle_s) {
    return CrosswalkCollisionState::EgoPassFirst;
  }
  const double later_extra = previous == CrosswalkCollisionState::EgoPassLater
                                 ? 0.0 : parameters.ego_pass_later_additional_margin_s;
  if (time_to_vehicle_s + parameters.ego_pass_later_margin_s + later_extra <
      time_to_collision_s) {
    return CrosswalkCollisionState::EgoPassLater;
  }
  return CrosswalkCollisionState::Yield;
}

}  // planning 네임스페이스
