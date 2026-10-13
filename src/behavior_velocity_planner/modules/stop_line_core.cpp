// Copyright 2020 Tier IV, Inc.
// SPDX-License-Identifier: Apache-2.0
// 원본 정지선 scene.cpp의 정지점과 상태 계산을 이관했습니다.

#include "planning/stop_line_core.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace planning {
namespace {

struct Point2d {
  double x;
  double y;
};

// 원본 autoware_trajectory의 양 끝 자연 경계조건 3차 스플라인입니다.
double splineValue(
    const std::vector<double> &arc, const Trajectory &path,
    double query, bool x_coordinate) {
  const std::size_t n = arc.size() - 1;
  std::vector<double> value(arc.size());
  for (std::size_t i = 0; i < value.size(); ++i) {
    value[i] = x_coordinate ? path[i].x : path[i].y;
  }
  std::vector<double> h(n), alpha(n - 1), l(n + 1), mu(n + 1), z(n + 1);
  std::vector<double> b(n), c(n + 1), d(n);
  for (std::size_t i = 0; i < n; ++i) h[i] = arc[i + 1] - arc[i];
  for (std::size_t i = 1; i < n; ++i) {
    alpha[i - 1] = 3.0 * (value[i + 1] - value[i]) / h[i] -
                   3.0 * (value[i] - value[i - 1]) / h[i - 1];
  }
  l[0] = 1.0;
  mu[0] = z[0] = 0.0;
  for (std::size_t i = 1; i < n; ++i) {
    l[i] = 2.0 * (arc[i + 1] - arc[i - 1]) - h[i - 1] * mu[i - 1];
    mu[i] = h[i] / l[i];
    z[i] = (alpha[i - 1] - h[i - 1] * z[i - 1]) / l[i];
  }
  l[n] = 1.0;
  z[n] = c[n] = 0.0;
  for (std::size_t j = n; j-- > 0;) {
    c[j] = z[j] - mu[j] * c[j + 1];
    b[j] = (value[j + 1] - value[j]) / h[j] -
           h[j] * (c[j + 1] + 2.0 * c[j]) / 3.0;
    d[j] = (c[j + 1] - c[j]) / (3.0 * h[j]);
  }
  const auto upper = std::upper_bound(arc.begin(), arc.end(), query);
  const std::size_t segment = std::min<std::size_t>(
      std::distance(arc.begin(), upper) - 1, n - 1);
  const double delta = query - arc[segment];
  return value[segment] + b[segment] * delta +
         c[segment] * delta * delta + d[segment] * delta * delta * delta;
}

double cross(Point2d a, Point2d b) { return a.x * b.y - a.y * b.x; }

double length(const TrajectoryPoint &a, const TrajectoryPoint &b) {
  return std::hypot(b.x - a.x, b.y - a.y);
}

std::vector<double> arcs(const Trajectory &path) {
  std::vector<double> result(path.size(), 0.0);
  for (std::size_t i = 1; i < path.size(); ++i) {
    result[i] = result[i - 1] + length(path[i - 1], path[i]);
  }
  return result;
}

double closestArc(
    const Trajectory &path, const std::vector<double> &arc,
    double x, double y) {
  double nearest = std::numeric_limits<double>::infinity();
  double selected = 0.0;
  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    const double dx = path[i + 1].x - path[i].x;
    const double dy = path[i + 1].y - path[i].y;
    const double ds2 = dx * dx + dy * dy;
    if (ds2 <= 1.0e-12) continue;
    const double ratio = std::clamp(
        ((x - path[i].x) * dx + (y - path[i].y) * dy) / ds2, 0.0, 1.0);
    const double px = path[i].x + ratio * dx;
    const double py = path[i].y + ratio * dy;
    const double distance2 = (x - px) * (x - px) + (y - py) * (y - py);
    if (distance2 < nearest) {
      nearest = distance2;
      selected = arc[i] + ratio * std::sqrt(ds2);
    }
  }
  return selected;
}

std::optional<double> crossingArc(
    const Trajectory &path, const std::vector<double> &arc,
    StopLineSegment line, double extension) {
  const double line_length = std::hypot(line.x2 - line.x1, line.y2 - line.y1);
  if (line_length <= 1.0e-12) return std::nullopt;
  const double ux = (line.x2 - line.x1) / line_length;
  const double uy = (line.y2 - line.y1) / line_length;
  Point2d a{line.x1 - extension * ux, line.y1 - extension * uy};
  Point2d b{line.x2 + extension * ux, line.y2 + extension * uy};
  const Point2d direction{b.x - a.x, b.y - a.y};
  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    const Point2d start{path[i].x, path[i].y};
    const Point2d path_direction{
        path[i + 1].x - start.x, path[i + 1].y - start.y};
    const double denominator = cross(path_direction, direction);
    if (std::abs(denominator) <= 1.0e-12) continue;
    const Point2d delta{a.x - start.x, a.y - start.y};
    const double path_ratio = cross(delta, direction) / denominator;
    const double line_ratio = cross(delta, path_direction) / denominator;
    if (path_ratio >= 0.0 && path_ratio <= 1.0 &&
        line_ratio >= 0.0 && line_ratio <= 1.0) {
      return arc[i] + path_ratio * length(path[i], path[i + 1]);
    }
  }
  return std::nullopt;
}

Trajectory zeroAfter(
    const Trajectory &path, const std::vector<double> &arc,
    double stop_arc, const StopLineParameters &parameters) {
  Trajectory result = path;
  std::size_t first_zero = std::distance(
      arc.begin(), std::lower_bound(arc.begin(), arc.end(), stop_arc));
  if (stop_arc > 0.0 && stop_arc < arc.back()) {
    const auto upper = std::upper_bound(arc.begin(), arc.end(), stop_arc);
    const std::size_t index = std::distance(arc.begin(), upper);
    if (index < arc.size() && arc[index - 1] < stop_arc) {
      const double ratio =
          (stop_arc - arc[index - 1]) / (arc[index] - arc[index - 1]);
      auto point = path[index - 1];
      point.x = parameters.linear_insert
          ? point.x + (path[index].x - point.x) * ratio
          : splineValue(arc, path, stop_arc, true);
      point.y = parameters.linear_insert
          ? point.y + (path[index].y - point.y) * ratio
          : splineValue(arc, path, stop_arc, false);
      point.z += (path[index].z - point.z) * ratio;
      const Eigen::Quaterniond before(
          path[index - 1].orientation_w, path[index - 1].orientation_x,
          path[index - 1].orientation_y, path[index - 1].orientation_z);
      const Eigen::Quaterniond after(
          path[index].orientation_w, path[index].orientation_x,
          path[index].orientation_y, path[index].orientation_z);
      const auto orientation = before.slerp(
          parameters.copy_previous_orientation ? 0.0 : ratio, after);
      point.orientation_x = orientation.x();
      point.orientation_y = orientation.y();
      point.orientation_z = orientation.z();
      point.orientation_w = orientation.w();
      point.longitudinal_velocity_mps = 0.0F;
      result.insert(result.begin() + index, point);
      first_zero = index;
    }
  }
  for (std::size_t i = first_zero; i < result.size(); ++i) {
    result[i].longitudinal_velocity_mps = 0.0F;
  }
  return result;
}

}  // 익명 네임스페이스

StopLineCore::StopLineCore(StopLineSegment line, StopLineParameters parameters)
    : line_(line), parameters_(parameters) {
  const double values[] = {
      line.x1, line.y1, line.x2, line.y2, parameters.stop_margin_m,
      parameters.stop_duration_s, parameters.hold_stop_margin_m,
      parameters.front_offset_m, parameters.line_extension_m};
  for (double value : values) {
    if (!std::isfinite(value)) {
      throw std::invalid_argument("정지선 설정에는 유한한 값이 필요합니다");
    }
  }
  if (parameters.stop_duration_s < 0.0 ||
      parameters.hold_stop_margin_m < 0.0 ||
      parameters.front_offset_m < 0.0 || parameters.line_extension_m < 0.0) {
    throw std::invalid_argument("정지선 설정이 잘못되었습니다");
  }
}

StopLineResult StopLineCore::apply(
    const Trajectory &path, double ego_x, double ego_y,
    bool vehicle_stopped, double now_s) {
  if (!std::isfinite(ego_x) || !std::isfinite(ego_y) || !std::isfinite(now_s)) {
    throw std::invalid_argument("차량 위치와 시각에는 유한한 값이 필요합니다");
  }
  StopLineResult result;
  result.trajectory = path;
  result.state = state_;
  if (path.size() < 2) return result;

  const auto arc = arcs(path);
  for (std::size_t i = 1; i < arc.size(); ++i) {
    if (arc[i] <= arc[i - 1]) return result;
  }
  result.ego_arc_m = closestArc(path, arc, ego_x, ego_y);
  if (state_ == StopLineState::Approach) {
    const auto crossing = crossingArc(path, arc, line_, parameters_.line_extension_m);
    if (crossing) {
      const double stop = *crossing -
          (parameters_.front_offset_m + parameters_.stop_margin_m);
      if (stop >= 0.0) result.stop_arc_m = stop;
    }
  } else if (state_ == StopLineState::Stopped) {
    result.stop_arc_m = result.ego_arc_m;
  }
  if (!result.stop_arc_m) return result;

  result.trajectory = zeroAfter(path, arc, *result.stop_arc_m, parameters_);
  if (state_ == StopLineState::Approach &&
      *result.stop_arc_m - result.ego_arc_m < parameters_.hold_stop_margin_m &&
      vehicle_stopped) {
    state_ = StopLineState::Stopped;
    stopped_since_s_ = now_s;
  } else if (state_ == StopLineState::Stopped && stopped_since_s_ &&
             now_s - *stopped_since_s_ > parameters_.stop_duration_s) {
    state_ = StopLineState::Start;
    stopped_since_s_.reset();
  }
  result.state = state_;
  return result;
}

}  // planning 네임스페이스
