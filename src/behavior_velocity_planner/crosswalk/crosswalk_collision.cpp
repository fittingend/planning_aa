// Copyright 2020 Tier IV, Inc.
// SPDX-License-Identifier: Apache-2.0
// 예측 객체의 점유 다각형과 차량 통과 영역의 중첩을 계산합니다.

#include "planning/crosswalk_core.hpp"

#include <boost/geometry.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace planning {
namespace {
namespace bg = boost::geometry;
using Point = bg::model::d2::point_xy<double>;
using Polygon = bg::model::polygon<Point>;
using MultiPoint = bg::model::multi_point<Point>;

std::vector<double> arcs(const Trajectory &path) {
  std::vector<double> result(path.size(), 0.0);
  for (std::size_t i = 1; i < path.size(); ++i) {
    result[i] = result[i - 1] + std::hypot(
        path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
  }
  return result;
}

double nearestArc(const std::vector<CrosswalkPoint> &path,
                  const std::vector<double> &arc, CrosswalkPoint point) {
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

CrosswalkPose2d poseAt(
    const Trajectory &path, const std::vector<double> &arc, double query) {
  const auto upper = std::upper_bound(arc.begin(), arc.end(), query);
  const std::size_t next = std::min<std::size_t>(
      std::distance(arc.begin(), upper), path.size() - 1);
  const std::size_t previous = next - 1;
  const double ratio = (query - arc[previous]) /
                       (arc[next] - arc[previous]);
  const double yaw = std::atan2(path[next].y - path[previous].y,
                                path[next].x - path[previous].x);
  return {path[previous].x + ratio * (path[next].x - path[previous].x),
          path[previous].y + ratio * (path[next].y - path[previous].y), yaw};
}

void appendBox(MultiPoint &points, CrosswalkPose2d pose,
               double front, double back, double half_width) {
  const double c = std::cos(pose.yaw_rad);
  const double s = std::sin(pose.yaw_rad);
  for (double longitudinal : {front, -back}) {
    for (double lateral : {-half_width, half_width}) {
      points.emplace_back(pose.x + longitudinal * c - lateral * s,
                          pose.y + longitudinal * s + lateral * c);
    }
  }
}

Polygon hull(const std::vector<CrosswalkPose2d> &poses,
             std::size_t first, std::size_t last,
             double front, double back, double half_width) {
  MultiPoint points;
  for (std::size_t i = first; i <= last; ++i) {
    appendBox(points, poses[i], front, back, half_width);
  }
  Polygon polygon;
  bg::convex_hull(points, polygon);
  bg::correct(polygon);
  return polygon;
}

std::vector<CrosswalkPose2d> sparsePath(
    const Trajectory &path, const std::vector<double> &arc, double interval) {
  std::vector<CrosswalkPose2d> sparse;
  for (double query = 0.0; query < arc.back(); query += interval) {
    sparse.push_back(poseAt(path, arc, query));
  }
  if (sparse.empty() ||
      std::hypot(sparse.back().x - path.back().x,
                 sparse.back().y - path.back().y) > 0.05) {
    sparse.push_back(poseAt(path, arc, arc.back()));
  }
  return sparse;
}

}  // 익명 네임스페이스

std::optional<CrosswalkCollisionPoint> estimateCrosswalkCollision(
    const Trajectory &path, CrosswalkPoint ego,
    double entry_arc_m, double exit_arc_m,
    double ego_speed_mps, const CrosswalkPredictedObject &object,
    CrosswalkCollisionParameters parameters) {
  if (path.size() < 2 || object.predicted_paths.empty()) return std::nullopt;
  if (object.length_m <= 0.0 || object.width_m <= 0.0 ||
      parameters.vehicle_width_m <= 0.0 || parameters.sparse_interval_m <= 0.0 ||
      parameters.ego_min_assumed_speed_mps <= 0.0 ||
      parameters.object_min_assumed_speed_mps <= 0.0) {
    throw std::invalid_argument("횡단보도 충돌 계산의 크기 또는 속도가 잘못되었습니다");
  }
  const auto path_arc = arcs(path);
  if (path_arc.back() <= 1.0e-9) return std::nullopt;
  auto sparse = sparsePath(path, path_arc, parameters.sparse_interval_m);
  std::vector<CrosswalkPoint> sparse_points;
  sparse_points.reserve(sparse.size());
  for (const auto &pose : sparse) sparse_points.push_back({pose.x, pose.y});
  std::vector<double> sparse_arc(sparse.size(), 0.0);
  for (std::size_t i = 1; i < sparse.size(); ++i) {
    sparse_arc[i] = sparse_arc[i - 1] + std::hypot(
        sparse[i].x - sparse[i - 1].x, sparse[i].y - sparse[i - 1].y);
  }
  const double ego_arc = nearestArc(sparse_points, sparse_arc, ego);
  const double near = std::max(0.0, entry_arc_m - ego_arc - parameters.attention_margin_m);
  const double far = std::max(0.0, exit_arc_m - ego_arc + parameters.attention_margin_m);
  Polygon attention;
  for (std::size_t i = 0; i + 1 < sparse.size(); ++i) {
    if (sparse_arc[i + 1] - ego_arc < near ||
        far < sparse_arc[i] - ego_arc) continue;
    const auto segment = hull(sparse, i, i + 1,
        parameters.vehicle_front_m, parameters.vehicle_back_m,
        parameters.vehicle_width_m / 2.0);
    if (attention.outer().empty()) {
      attention = segment;
    } else {
      std::vector<Polygon> merged;
      bg::union_(attention, segment, merged);
      if (!merged.empty()) attention = merged.front();
    }
  }
  if (attention.outer().empty()) return std::nullopt;

  std::optional<CrosswalkCollisionPoint> nearest;
  const double object_speed = std::max(
      parameters.object_min_assumed_speed_mps,
      std::hypot(object.velocity_x_mps, object.velocity_y_mps));
  for (const auto &predicted : object.predicted_paths) {
    if (predicted.empty()) continue;
    std::size_t start = 0;
    bool started = false;
    for (std::size_t i = 0; i < predicted.size(); ++i) {
      const auto one_step = hull(predicted, i, i,
          object.width_m / 2.0, object.width_m / 2.0,
          object.length_m / 2.0);
      std::vector<Polygon> intersections;
      bg::intersection(one_step, attention, intersections);
      if (!intersections.empty()) {
        if (!started) { start = i; started = true; }
        if (i + 1 < predicted.size()) continue;
      }
      if (!started) continue;
      const std::size_t first = start == 0 ? 0 : start - 1;
      const std::size_t last = std::min(i + 1, predicted.size() - 1);
      const auto swept = hull(predicted, first, last,
          object.width_m / 2.0, object.width_m / 2.0,
          object.length_m / 2.0);
      intersections.clear();
      bg::intersection(swept, attention, intersections);
      started = false;
      if (intersections.empty()) continue;
      Point centroid{0.0, 0.0};
      bg::centroid(intersections.front(), centroid);
      const CrosswalkPoint collision{centroid.x(), centroid.y()};
      const double ego_dist = nearestArc(sparse_points, sparse_arc, collision) - ego_arc;
      if (ego_dist < near || ego_dist > far) continue;
      std::vector<CrosswalkPoint> object_points;
      std::vector<double> object_arc(predicted.size(), 0.0);
      for (std::size_t j = 0; j < predicted.size(); ++j) {
        object_points.push_back({predicted[j].x, predicted[j].y});
        if (j) object_arc[j] = object_arc[j - 1] + std::hypot(
            predicted[j].x - predicted[j - 1].x,
            predicted[j].y - predicted[j - 1].y);
      }
      const double object_dist = predicted.size() == 1 ? 0.0 :
          nearestArc(object_points, object_arc, collision);
      if (!nearest || ego_dist < nearest->ego_distance_m) {
        nearest = CrosswalkCollisionPoint{
            collision, ego_dist, object_dist,
            std::max(0.0, ego_dist + parameters.vehicle_back_m) /
                std::max(ego_speed_mps, parameters.ego_min_assumed_speed_mps),
            std::max(0.0, object_dist) / object_speed};
      }
      break;
    }
  }
  return nearest;
}

}  // planning 네임스페이스
