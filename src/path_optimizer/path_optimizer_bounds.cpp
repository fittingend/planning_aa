// Copyright 2023 TIER IV, Inc.
// SPDX-License-Identifier: Apache-2.0
// 원본 경로 최적화의 횡단선 교차를 사용한 차선 경계 거리입니다.

#include "planning/path_optimizer_bounds.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace planning {
namespace {

double cross(double ax, double ay, double bx, double by) {
  return ax * by - ay * bx;
}

}  // 익명 네임스페이스

double pathOptimizerLateralBound(
    const TrajectoryPoint &reference,
    const std::vector<PathOptimizerPoint2D> &polyline,
    const double clearance, const bool left) {
  if (polyline.size() < 2 || !std::isfinite(clearance) || clearance < 0.0) {
    throw std::invalid_argument("경로 최적화 차선 경계 입력이 유효하지 않습니다");
  }
  const double yaw = std::atan2(
      2.0 * (reference.orientation_w * reference.orientation_z +
             reference.orientation_x * reference.orientation_y),
      1.0 - 2.0 * (reference.orientation_y * reference.orientation_y +
                   reference.orientation_z * reference.orientation_z));
  const double normal_x = -std::sin(yaw);
  const double normal_y = std::cos(yaw);
  const double min_offset = left ? -5.0 : 5.0;
  const double max_offset = -min_offset;
  const double ray_x = (max_offset - min_offset) * normal_x;
  const double ray_y = (max_offset - min_offset) * normal_y;
  const double origin_x = reference.x + min_offset * normal_x;
  const double origin_y = reference.y + min_offset * normal_y;
  double closest = left ? 5.0 : -5.0;
  for (std::size_t i = 0; i + 1 < polyline.size(); ++i) {
    const double dx = polyline[i + 1].x - polyline[i].x;
    const double dy = polyline[i + 1].y - polyline[i].y;
    const double determinant = cross(ray_x, ray_y, dx, dy);
    if (std::abs(determinant) < 1.0e-12) continue;
    const double to_bound_x = polyline[i].x - origin_x;
    const double to_bound_y = polyline[i].y - origin_y;
    const double ray_ratio = cross(to_bound_x, to_bound_y, dx, dy) / determinant;
    const double bound_ratio = cross(to_bound_x, to_bound_y, ray_x, ray_y) / determinant;
    if (ray_ratio < 0.0 || ray_ratio > 1.0 ||
        bound_ratio < 0.0 || bound_ratio > 1.0) continue;
    const double lateral = min_offset + ray_ratio * (max_offset - min_offset);
    const double candidate = left ? lateral - clearance : lateral + clearance;
    if (std::abs(candidate) < std::abs(closest)) closest = candidate;
  }
  return closest;
}

std::vector<PathOptimizerBound> pathOptimizerCenterBounds(
    const Trajectory &reference,
    const std::vector<PathOptimizerPoint2D> &left_polyline,
    const std::vector<PathOptimizerPoint2D> &right_polyline,
    const double vehicle_width, const double soft_clearance,
    const double interval) {
  if (reference.size() < 2 || !std::isfinite(vehicle_width) ||
      vehicle_width <= 0.0 || !std::isfinite(soft_clearance) ||
      soft_clearance < 0.0 || !std::isfinite(interval) || interval <= 0.0) {
    throw std::invalid_argument("경로 최적화 기준 경로 또는 차량 폭이 유효하지 않습니다");
  }
  const double clearance = vehicle_width / 2.0 + soft_clearance;
  const std::size_t minimum_index = std::min<std::size_t>(
      static_cast<std::size_t>(std::ceil(1.0 / interval)), reference.size() - 1);
  std::vector<PathOptimizerBound> bounds;
  bounds.reserve(reference.size());
  for (std::size_t i = 0; i < reference.size(); ++i) {
    const auto &point = reference[std::max(minimum_index, i)];
    const double left = pathOptimizerLateralBound(
        point, left_polyline, clearance, true);
    const double right = pathOptimizerLateralBound(
        point, right_polyline, clearance, false);
    bounds.push_back({right, left});
  }
  return bounds;
}

void keepPathOptimizerMinimumBoundsWidth(
    std::vector<PathOptimizerBound> &bounds,
    const std::vector<double> &curvature, const double minimum_width,
    const double vehicle_width, const double maximum_longitudinal_offset) {
  if (bounds.empty() || bounds.size() != curvature.size() ||
      !std::isfinite(minimum_width) || minimum_width < 0.0 ||
      !std::isfinite(vehicle_width) || vehicle_width <= 0.0 ||
      !std::isfinite(maximum_longitudinal_offset) ||
      maximum_longitudinal_offset < 0.0) {
    throw std::invalid_argument("경로 최적화 최소 경계 폭 입력이 유효하지 않습니다");
  }
  std::vector<double> required(bounds.size());
  for (std::size_t i = 0; i < bounds.size(); ++i) {
    double value = std::abs(curvature[i]);
    if (i > 0) value = std::max(value, std::abs(curvature[i - 1]));
    if (i + 1 < bounds.size()) value = std::max(value, std::abs(curvature[i + 1]));
    const double radius = 1.0 / value;
    const double extra = std::hypot(maximum_longitudinal_offset,
        radius + vehicle_width / 2.0) - radius - vehicle_width / 2.0;
    required[i] = minimum_width + (std::isfinite(extra) ? extra : 0.0);
  }
  std::vector<bool> upper_out(bounds.size()), lower_out(bounds.size());
  for (std::size_t i = 0; i < bounds.size(); ++i) {
    upper_out[i] = bounds[i].upper < 0.0;
    lower_out[i] = bounds[i].lower > 0.0;
  }
  const auto original = bounds;
  for (std::size_t start = 0; start < bounds.size();) {
    if (!lower_out[start]) { ++start; continue; }
    std::size_t end = start;
    while (end + 1 < bounds.size() && lower_out[end + 1]) ++end;
    std::size_t first_overlap = bounds.size(), last_overlap = bounds.size();
    for (std::size_t i = start; i <= end; ++i) {
      if (upper_out[i]) {
        first_overlap = std::min(first_overlap, i);
        last_overlap = i;
        const double center = (original[i].lower + original[i].upper) / 2.0;
        bounds[i].upper = std::max(bounds[i].upper, center + required[i] / 2.0);
        bounds[i].lower = std::min(bounds[i].lower, center - required[i] / 2.0);
      } else {
        bounds[i].lower = std::min(bounds[i].lower,
                                    original[i].upper - required[i]);
      }
    }
    if (first_overlap < bounds.size()) {
      for (std::size_t i = start; i < first_overlap; ++i)
        bounds[i].lower = std::min(bounds[i].lower, bounds[first_overlap].lower);
      for (std::size_t i = last_overlap + 1; i <= end; ++i)
        bounds[i].lower = std::min(bounds[i].lower, bounds[last_overlap].lower);
    }
    start = end + 1;
  }
  for (std::size_t start = 0; start < bounds.size();) {
    if (!upper_out[start]) { ++start; continue; }
    std::size_t end = start;
    while (end + 1 < bounds.size() && upper_out[end + 1]) ++end;
    std::size_t first_overlap = bounds.size(), last_overlap = bounds.size();
    for (std::size_t i = start; i <= end; ++i) {
      if (lower_out[i]) {
        first_overlap = std::min(first_overlap, i);
        last_overlap = i;
        const double center = (original[i].lower + original[i].upper) / 2.0;
        bounds[i].upper = std::max(bounds[i].upper, center + required[i] / 2.0);
        bounds[i].lower = std::min(bounds[i].lower, center - required[i] / 2.0);
      } else {
        bounds[i].upper = std::max(bounds[i].upper,
                                    original[i].lower + required[i]);
      }
    }
    if (first_overlap < bounds.size()) {
      for (std::size_t i = start; i < first_overlap; ++i)
        bounds[i].upper = std::max(bounds[i].upper, bounds[first_overlap].upper);
      for (std::size_t i = last_overlap + 1; i <= end; ++i)
        bounds[i].upper = std::max(bounds[i].upper, bounds[last_overlap].upper);
    }
    start = end + 1;
  }
}

std::vector<PathOptimizerBound> extendPathOptimizerViolatedBounds(
    const std::vector<PathOptimizerBound> &bounds,
    const double maximum_margin, const double interval) {
  if (!std::isfinite(maximum_margin) || maximum_margin < 0.0 ||
      !std::isfinite(interval) || interval <= 0.0) {
    throw std::invalid_argument("경로 최적화 경계 확장 길이가 유효하지 않습니다");
  }
  auto extended = bounds;
  const int count = static_cast<int>(bounds.size());
  const int margin = static_cast<int>(std::floor(maximum_margin / interval));
  for (int i = 0; i + 1 < count; ++i) {
    for (int j = 0; j <= margin; ++j) {
      const int before = std::clamp(i - j, 0, count - 1);
      const int after = std::clamp(i + j, 0, count - 1);
      if (bounds[i].lower <= 0.0 && bounds[i + 1].lower >= 0.0)
        extended[before].lower = bounds[i + 1].lower;
      if (bounds[i].upper >= 0.0 && bounds[i + 1].upper <= 0.0)
        extended[before].upper = bounds[i + 1].upper;
      if (bounds[i].lower >= 0.0 && bounds[i + 1].lower <= 0.0)
        extended[after].lower = bounds[i].lower;
      if (bounds[i].upper <= 0.0 && bounds[i + 1].upper >= 0.0)
        extended[after].upper = bounds[i].upper;
    }
  }
  return extended;
}

void reusePathOptimizerPreviousBounds(
    std::vector<PathOptimizerBound> &bounds,
    const PathOptimizerReferenceGeometry &geometry,
    const TrajectoryPoint &ego, const double ego_speed,
    const PathOptimizerReferenceGeometry &previous_geometry,
    const std::vector<PathOptimizerBound> &previous_bounds,
    const double maximum_fixing_time, const double interval,
    const double nearest_distance, const double nearest_yaw) {
  if (bounds.empty() || bounds.size() != geometry.points.size() ||
      previous_bounds.empty() ||
      previous_bounds.size() != previous_geometry.points.size() ||
      !std::isfinite(ego_speed) || !std::isfinite(maximum_fixing_time) ||
      maximum_fixing_time < 0.0 || !std::isfinite(interval) ||
      interval <= 0.0 || !std::isfinite(nearest_distance) ||
      nearest_distance < 0.0 || !std::isfinite(nearest_yaw) ||
      nearest_yaw < 0.0) {
    throw std::invalid_argument("경로 최적화 이전 경계 입력이 유효하지 않습니다");
  }
  const auto yaw = [](const TrajectoryPoint &point) {
    return std::atan2(
        2.0 * (point.orientation_w * point.orientation_z +
               point.orientation_x * point.orientation_y),
        1.0 - 2.0 * (point.orientation_y * point.orientation_y +
                     point.orientation_z * point.orientation_z));
  };
  const auto nearest = [&](const Trajectory &points,
                           const TrajectoryPoint &pose) {
    const double distance_limit_squared = nearest_distance * nearest_distance;
    for (int pass = 0; pass < 3; ++pass) {
      std::size_t best_index = 0;
      double best_distance_squared = std::numeric_limits<double>::infinity();
      bool found = false;
      for (std::size_t i = 0; i < points.size(); ++i) {
        const double dx = points[i].x - pose.x;
        const double dy = points[i].y - pose.y;
        const double distance_squared = dx * dx + dy * dy;
        const bool outside =
            (pass < 2 && distance_squared > distance_limit_squared) ||
            (pass == 0 && std::abs(std::remainder(
                yaw(points[i]) - yaw(pose), 2.0 * std::acos(-1.0))) >
                nearest_yaw);
        if (outside) {
          if (found && pass < 2) break;
          continue;
        }
        if (distance_squared >= best_distance_squared) continue;
        best_distance_squared = distance_squared;
        best_index = i;
        found = true;
      }
      if (found) return best_index;
    }
    return std::size_t{0};
  };
  const std::size_t previous_front = nearest(
      previous_geometry.points, geometry.points.front());
  const std::size_t ego_index = nearest(geometry.points, ego);
  const int fixing_points = static_cast<int>(
      std::floor(ego_speed * maximum_fixing_time / interval));
  if (fixing_points <= 0) return;
  const std::size_t count = std::min(
      ego_index + static_cast<std::size_t>(fixing_points), bounds.size());
  for (std::size_t i = 0; i < count; ++i) {
    bounds[i] = previous_bounds[std::min(
        previous_front + i, previous_bounds.size() - 1)];
  }
}

void attachPathOptimizerVehicleBounds(
    const PathOptimizerReferenceGeometry &geometry,
    const std::vector<PathOptimizerBound> &center_bounds,
    const std::vector<PathOptimizerCircle> &circles,
    std::vector<PathOptimizerReference> &references) {
  const std::size_t count = geometry.points.size();
  if (count < 2 || center_bounds.size() != count ||
      references.size() != count || circles.empty()) {
    throw std::invalid_argument("경로 최적화 차량 원 경계 입력이 유효하지 않습니다");
  }
  std::vector<double> offsets;
  for (const auto &circle : circles) offsets.push_back(circle.longitudinal_offset);
  const auto poses = samplePathOptimizerSplineOffsets(geometry.points, offsets);
  std::vector<double> arcs(count, 0.0);
  for (std::size_t i = 1; i < count; ++i)
    arcs[i] = arcs[i - 1] + std::hypot(geometry.points[i].x - geometry.points[i - 1].x,
                                     geometry.points[i].y - geometry.points[i - 1].y);
  for (std::size_t i = 0; i < count; ++i) {
    auto &reference = references[i];
    reference.beta.clear();
    reference.circle_bounds.clear();
    const auto &point = geometry.points[i];
    const double yaw = std::atan2(
        2.0 * (point.orientation_w * point.orientation_z +
               point.orientation_x * point.orientation_y),
        1.0 - 2.0 * (point.orientation_y * point.orientation_y +
                     point.orientation_z * point.orientation_z));
    for (std::size_t c = 0; c < circles.size(); ++c) {
      const auto &pose = poses[i][c];
      reference.beta.push_back(yaw - pose.yaw);
      const double dx = pose.x - point.x;
      const double dy = pose.y - point.y;
      const double offset_y = dx * std::sin(pose.yaw) - dy * std::cos(pose.yaw);
      const double query = arcs[i] + circles[c].longitudinal_offset;
      const auto next = std::upper_bound(arcs.begin(), arcs.end(), query);
      const std::size_t upper = std::min<std::size_t>(
          static_cast<std::size_t>(next - arcs.begin()), count - 1);
      const std::size_t lower = std::clamp<std::size_t>(
          upper == 0 ? 0 : upper - 1, 0, count - 2);
      const double ratio = std::clamp(
          (query - arcs[lower]) / (arcs[lower + 1] - arcs[lower]), 0.0, 1.0);
      const PathOptimizerBound interpolated{
          center_bounds[lower].lower + ratio *
              (center_bounds[lower + 1].lower - center_bounds[lower].lower),
          center_bounds[lower].upper + ratio *
              (center_bounds[lower + 1].upper - center_bounds[lower].upper)};
      reference.circle_bounds.push_back(
          {interpolated.lower - offset_y, interpolated.upper - offset_y});
    }
  }
}

}  // planning 네임스페이스
