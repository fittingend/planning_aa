// Copyright 2023 TIER IV, Inc.
// SPDX-License-Identifier: Apache-2.0
// 원본 MPTOptimizer::updateExtraPoints의 계산 부분입니다.

#include "planning/path_optimizer_reference_data.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace planning {

std::vector<PathOptimizerReference> buildPathOptimizerReferenceData(
    const PathOptimizerReferenceGeometry &geometry,
    const std::vector<PathOptimizerBound> &center_bounds,
    const PathOptimizerAvoidanceParameters &p,
    const PathOptimizerReferenceGeometry *previous_geometry,
    const std::vector<PathOptimizerReference> *previous_references) {
  const std::size_t count = geometry.points.size();
  if (count < 2 || geometry.curvature.size() != count ||
      geometry.arc_length_to_next.size() != count ||
      (!geometry.fixed_states.empty() && geometry.fixed_states.size() != count) ||
      center_bounds.size() != count ||
      !std::isfinite(p.wheelbase) || p.wheelbase <= 0.0 ||
      !std::isfinite(p.maximum_cost_distance) || p.maximum_cost_distance <= 0.0 ||
      !std::isfinite(p.cost_margin) || p.cost_margin < 0.0 ||
      !std::isfinite(p.band_length) || p.band_length < 0.0 ||
      !std::isfinite(p.decrease_per_point) || p.decrease_per_point <= 0.0 ||
      !std::isfinite(p.interval) || p.interval <= 0.0 ||
      (previous_geometry != nullptr) != (previous_references != nullptr) ||
      (previous_geometry &&
       previous_geometry->points.size() != previous_references->size())) {
    throw std::invalid_argument("경로 최적화 추가 기준점 입력이 유효하지 않습니다");
  }
  std::vector<PathOptimizerReference> references(count);
  for (std::size_t i = 0; i < count; ++i) {
    auto &reference = references[i];
    reference.arc_length_to_next = geometry.arc_length_to_next[i];
    reference.curvature = geometry.curvature[i];
    if (!geometry.fixed_states.empty() && geometry.fixed_states[i]) {
      reference.fixed_state = Eigen::Vector2d(
          (*geometry.fixed_states[i])[0], (*geometry.fixed_states[i])[1]);
    }
    const auto &point = geometry.points[i];
    double accumulated = 0.0;
    std::size_t front_index = count - 1;
    for (std::size_t j = i; j < count; ++j) {
      accumulated += geometry.arc_length_to_next[j];
      if (p.wheelbase < accumulated) { front_index = j; break; }
    }
    const auto &front = geometry.points[front_index];
    const double dx = front.x - point.x;
    const double dy = front.y - point.y;
    const double yaw = std::atan2(
        2.0 * (point.orientation_w * point.orientation_z +
               point.orientation_x * point.orientation_y),
        1.0 - 2.0 * (point.orientation_y * point.orientation_y +
                     point.orientation_z * point.orientation_z));
    const double front_yaw = std::hypot(dx, dy) < 1.0e-3 ? yaw :
        std::atan2(dy, dx);
    reference.alpha = std::remainder(front_yaw - yaw,
                                     2.0 * 3.141592653589793);
  }
  const int band = static_cast<int>(std::floor(p.band_length / p.interval));
  for (std::size_t i = 0; i < count; ++i) {
    const auto &bound = center_bounds[i];
    const double negative_cost = std::min(
        -bound.lower - p.cost_margin,
        bound.upper - p.cost_margin);
    if (negative_cost >= 0.0) continue;
    const double cost = std::clamp(
        -negative_cost / p.maximum_cost_distance, 0.0, 1.0);
    for (int distance = -band; distance <= band; ++distance) {
      const int index = static_cast<int>(i) + distance;
      if (index >= 0 && index < static_cast<int>(count)) {
        references[static_cast<std::size_t>(index)].avoidance_cost = cost;
      }
    }
  }
  for (int i = 0; i < static_cast<int>(count); ++i) {
    const double base = references[static_cast<std::size_t>(i)].avoidance_cost;
    if (base <= 0.0) continue;
    const int spread = static_cast<int>(std::floor(base / p.decrease_per_point));
    for (int distance = -spread; distance <= spread; ++distance) {
      const int index = i + distance;
      if (index < 0 || index >= static_cast<int>(count)) continue;
      auto &cost = references[static_cast<std::size_t>(index)].avoidance_cost;
      cost = std::clamp(std::max(cost, base -
          std::abs(distance) * p.decrease_per_point), 0.0, 1.0);
    }
  }
  if (previous_geometry) {
    for (std::size_t i = 0; i < count; ++i) {
      std::size_t nearest = 0;
      double best = std::numeric_limits<double>::infinity();
      for (std::size_t j = 0; j < previous_geometry->points.size(); ++j) {
        const double distance = std::hypot(
            geometry.points[i].x - previous_geometry->points[j].x,
            geometry.points[i].y - previous_geometry->points[j].y);
        if (distance < best) { best = distance; nearest = j; }
      }
      if (best <= p.interval / 2.0) {
        references[i].avoidance_cost = std::max(
            references[i].avoidance_cost,
            (*previous_references)[nearest].avoidance_cost);
      }
    }
  }
  return references;
}

}  // planning 네임스페이스
