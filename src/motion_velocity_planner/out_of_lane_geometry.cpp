// Copyright 2024 TIER IV, Inc.
// SPDX-License-Identifier: Apache-2.0
// 차로 겹침 형상과 예측 객체의 시간별 충돌에서 감속 후보를 계산합니다.

#include "planning/out_of_lane_core.hpp"

#include <boost/geometry.hpp>

#include <algorithm>
#include <cmath>
#include <set>
#include <stdexcept>
#include <vector>

namespace planning {
namespace {
namespace bg = boost::geometry;
using Point = bg::model::d2::point_xy<double>;
using Polygon = bg::model::polygon<Point>;

struct OutsidePoint {
  std::size_t trajectory_index = 0;
  std::vector<std::size_t> lane_indexes;
  std::vector<Polygon> overlaps;
  std::set<double> collision_times;
};

Polygon polygonFromPoints(const OutOfLanePolygon &points) {
  Polygon polygon;
  if (points.size() < 3) return polygon;
  for (const auto &point : points)
    polygon.outer().emplace_back(point.x, point.y);
  bg::correct(polygon);
  return polygon;
}

double yawOf(const TrajectoryPoint &point) {
  return std::atan2(
      2.0 * (point.orientation_w * point.orientation_z +
             point.orientation_x * point.orientation_y),
      1.0 - 2.0 * (point.orientation_y * point.orientation_y +
                   point.orientation_z * point.orientation_z));
}

Polygon footprint(const TrajectoryPoint &pose, double front, double rear,
                  double left, double right) {
  const double c = std::cos(yawOf(pose));
  const double s = std::sin(yawOf(pose));
  Polygon polygon;
  for (const auto &offset : std::vector<MotionVelocityPoint2D>{
           {front, left}, {front, right}, {rear, right}, {rear, left}}) {
    polygon.outer().emplace_back(pose.x + c * offset.x - s * offset.y,
                                 pose.y + s * offset.x + c * offset.y);
  }
  bg::correct(polygon);
  return polygon;
}

std::vector<double> arcLengths(const Trajectory &path) {
  std::vector<double> arcs(path.size(), 0.0);
  for (std::size_t i = 1; i < path.size(); ++i)
    arcs[i] = arcs[i - 1] + std::hypot(path[i].x - path[i - 1].x,
                                      path[i].y - path[i - 1].y);
  return arcs;
}

TrajectoryPoint poseAtArc(const Trajectory &path,
                          const std::vector<double> &arcs, double arc) {
  if (arc <= 0.0) return path.front();
  if (arc >= arcs.back()) return path.back();
  const auto upper = std::upper_bound(arcs.begin(), arcs.end(), arc);
  const std::size_t index = std::distance(arcs.begin(), upper) - 1;
  const double segment = arcs[index + 1] - arcs[index];
  const double ratio = segment < 1e-9 ? 0.0 :
      (arc - arcs[index]) / segment;
  TrajectoryPoint pose = path[index];
  pose.x += ratio * (path[index + 1].x - path[index].x);
  pose.y += ratio * (path[index + 1].y - path[index].y);
  const double first_yaw = yawOf(path[index]);
  const double yaw = first_yaw + ratio * std::remainder(
      yawOf(path[index + 1]) - first_yaw, 2.0 * 3.141592653589793);
  pose.orientation_x = 0.0;
  pose.orientation_y = 0.0;
  pose.orientation_z = std::sin(yaw / 2.0);
  pose.orientation_w = std::cos(yaw / 2.0);
  return pose;
}

bool avoidsAll(const Polygon &footprint,
               const std::vector<Polygon> &lanes,
               const std::vector<std::size_t> &indexes) {
  return std::all_of(indexes.begin(), indexes.end(), [&](std::size_t i) {
    return bg::disjoint(footprint, lanes[i]);
  });
}
}  // 익명 네임스페이스

std::optional<MotionVelocityPoint2D> findOutOfLaneSlowdownCandidate(
    const Trajectory &path, const std::vector<double> &time_from_start,
    const std::vector<OutOfLanePolygon> &other_lane_polygons,
    const std::vector<OutOfLanePredictedPath> &objects,
    const double minimum_stop_arc,
    const OutOfLaneParameters &decision,
    const OutOfLaneGeometryParameters &p) {
  if (path.size() < 2 || path.size() != time_from_start.size() ||
      !std::isfinite(minimum_stop_arc) || minimum_stop_arc < 0.0 ||
      p.position_precision <= 0.0)
    throw std::invalid_argument("차로 밖 주행 형상 입력이 유효하지 않습니다");
  std::vector<Polygon> lanes;
  lanes.reserve(other_lane_polygons.size());
  for (const auto &points : other_lane_polygons)
    lanes.push_back(polygonFromPoints(points));
  std::vector<OutsidePoint> outside;
  for (std::size_t i = 0; i < path.size(); ++i) {
    const auto ego = footprint(
        path[i], p.ego_front_offset + p.extra_front_offset,
        p.ego_rear_offset - p.extra_rear_offset,
        p.ego_left_offset + p.extra_left_offset,
        p.ego_right_offset - p.extra_right_offset);
    OutsidePoint entry;
    entry.trajectory_index = i;
    for (std::size_t j = 0; j < lanes.size(); ++j) {
      if (bg::is_empty(lanes[j])) continue;
      std::vector<Polygon> intersections;
      bg::intersection(ego, lanes[j], intersections);
      if (!intersections.empty()) {
        entry.lane_indexes.push_back(j);
        entry.overlaps.insert(entry.overlaps.end(),
                              intersections.begin(), intersections.end());
      }
    }
    if (!entry.overlaps.empty()) outside.push_back(std::move(entry));
  }
  if (outside.empty()) return std::nullopt;
  for (const auto &object : objects) {
    if (object.confidence < p.minimum_object_confidence ||
        object.time_step <= 0.0 || object.object_length <= 0.0 ||
        object.object_width <= 0.0) continue;
    double time = object.time_step;
    for (const auto &pose : object.poses) {
      time += object.time_step;
      const auto shape = footprint(
          pose, object.object_length / 2.0,
          -object.object_length / 2.0,
          object.object_width / 2.0, -object.object_width / 2.0);
      for (auto &entry : outside) {
        if (std::any_of(entry.overlaps.begin(), entry.overlaps.end(),
                        [&](const auto &overlap) {
                          return !bg::disjoint(shape, overlap);
                        })) entry.collision_times.insert(time);
      }
    }
  }
  const auto chosen = std::find_if(outside.begin(), outside.end(),
      [&](const auto &entry) {
        const std::vector<double> times(entry.collision_times.begin(),
                                        entry.collision_times.end());
        return classifyOutOfLaneCollisionTimes(
            times, time_from_start[entry.trajectory_index], decision).avoid;
      });
  if (chosen == outside.end()) return std::nullopt;
  const auto arcs = arcLengths(path);
  const double first_outside_arc = arcs[outside.front().trajectory_index];
  for (int expansion = 2; expansion >= 0; --expansion) {
    const double front = p.ego_front_offset +
        (expansion > 0 ? p.extra_front_offset : 0.0) +
        (expansion == 2 ? p.front_distance_buffer : 0.0);
    const double rear = p.ego_rear_offset -
        (expansion > 0 ? p.extra_rear_offset : 0.0);
    const double left = p.ego_left_offset +
        (expansion > 0 ? p.extra_left_offset : 0.0) +
        (expansion == 2 ? p.lateral_distance_buffer : 0.0);
    const double right = p.ego_right_offset -
        (expansion > 0 ? p.extra_right_offset : 0.0) -
        (expansion == 2 ? p.lateral_distance_buffer : 0.0);
    double from = minimum_stop_arc;
    double to = first_outside_arc;
    bool avoiding = false;
    TrajectoryPoint last;
    while (to - from > p.position_precision) {
      const double middle = from + 0.5 * (to - from);
      last = poseAtArc(path, arcs, middle);
      avoiding = avoidsAll(
          footprint(last, front, rear, left, right),
          lanes, chosen->lane_indexes);
      if (avoiding) from = middle;
      else to = middle;
    }
    if (avoiding) return MotionVelocityPoint2D{last.x, last.y};
  }
  for (double arc = first_outside_arc - p.position_precision;
       arc >= minimum_stop_arc; arc -= p.position_precision) {
    const auto pose = poseAtArc(path, arcs, arc);
    const auto ego = footprint(
        pose, p.ego_front_offset + p.extra_front_offset +
                  p.front_distance_buffer,
        p.ego_rear_offset - p.extra_rear_offset,
        p.ego_left_offset + p.extra_left_offset +
                  p.lateral_distance_buffer,
        p.ego_right_offset - p.extra_right_offset -
                  p.lateral_distance_buffer);
    if (std::any_of(chosen->overlaps.begin(), chosen->overlaps.end(),
                    [&](const auto &overlap) {
                      return !bg::disjoint(ego, overlap);
                    })) return MotionVelocityPoint2D{pose.x, pose.y};
  }
  return std::nullopt;
}

}  // planning 네임스페이스
