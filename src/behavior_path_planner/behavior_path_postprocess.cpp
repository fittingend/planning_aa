// Copyright 2020 Tier IV, Inc.
// SPDX-License-Identifier: Apache-2.0
// 원본 path_utilization.cpp와 resamplePath의 출력 경로 계산을 이관했습니다.

#include "planning/behavior_path_postprocess.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>
#include <vector>

namespace planning {
namespace {

class AkimaSpline {
 public:
  AkimaSpline(const std::vector<double> &keys, std::vector<double> values)
      : keys_(keys), values_(std::move(values)) {
    const std::size_t n = keys.size() - 1;
    std::vector<double> slope(n), tangent(n + 1);
    for (std::size_t i = 0; i < n; ++i) {
      slope[i] = (values_[i + 1] - values_[i]) /
                 (keys[i + 1] - keys[i]);
    }
    for (std::size_t i = 0; i <= n; ++i) {
      if (i == 0) {
        tangent[i] = slope.front();
      } else if (i == n) {
        tangent[i] = slope.back();
      } else if (i == 1 || i == n - 1) {
        tangent[i] = (slope[i - 1] + slope[i]) / 2.0;
      } else {
        const double left = std::abs(slope[i + 1] - slope[i]);
        const double right = std::abs(slope[i - 1] - slope[i - 2]);
        const double denominator = left + right;
        tangent[i] = denominator < 1.0e-5
            ? (slope[i - 1] + slope[i]) / 2.0
            : (left * slope[i - 1] + right * slope[i]) / denominator;
      }
    }
    a_.resize(n);
    b_.resize(n);
    c_.resize(n);
    for (std::size_t i = 0; i < n; ++i) {
      const double h = keys[i + 1] - keys[i];
      a_[i] = (tangent[i] + tangent[i + 1] - 2.0 * slope[i]) / (h * h);
      b_[i] = (3.0 * slope[i] - 2.0 * tangent[i] - tangent[i + 1]) / h;
      c_[i] = tangent[i];
    }
  }

  double value(double query) const {
    const auto upper = std::upper_bound(keys_.begin(), keys_.end(), query);
    const std::size_t segment = std::clamp<std::size_t>(
        std::distance(keys_.begin(), upper) - 1, 0, b_.size() - 1);
    const double delta = query - keys_[segment];
    return values_[segment] +
           (c_[segment] + (b_[segment] + a_[segment] * delta) * delta) * delta;
  }

 private:
  const std::vector<double> &keys_;
  std::vector<double> values_;
  std::vector<double> a_, b_, c_;
};

Trajectory filterClosePoints(const Trajectory &path) {
  Trajectory filtered;
  for (const auto &point : path) {
    if (filtered.empty() ||
        std::hypot(point.x - filtered.back().x, point.y - filtered.back().y) > 0.01) {
      filtered.push_back(point);
    } else {
      filtered.back().longitudinal_velocity_mps = std::min(
          filtered.back().longitudinal_velocity_mps,
          point.longitudinal_velocity_mps);
    }
  }
  return filtered;
}

std::vector<double> arcLengths(const Trajectory &path) {
  std::vector<double> arcs(path.size(), 0.0);
  for (std::size_t i = 1; i < path.size(); ++i) {
    arcs[i] = arcs[i - 1] +
        std::hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
  }
  return arcs;
}

Trajectory interpolate(const Trajectory &path, double length, double interval) {
  if (path.size() < 2) return path;
  const auto arc = arcLengths(path);
  const double path_length = std::min(length, arc.back());
  std::vector<double> query;
  for (double s : arc) {
    query.push_back(s);
    if (s > path_length) break;
  }
  if (std::abs(query.back() - path_length) < 0.01) {
    query.back() = path_length;
  } else {
    query.push_back(path_length);
  }
  for (double s = 0.0; s < path_length; s += interval) {
    const bool close = std::any_of(query.begin(), query.end(),
        [s](double existing) { return std::abs(existing - s) < 0.01; });
    if (!close) query.push_back(s);
  }
  std::sort(query.begin(), query.end());

  std::vector<double> x, y;
  x.reserve(path.size());
  y.reserve(path.size());
  for (const auto &point : path) {
    x.push_back(point.x);
    y.push_back(point.y);
  }
  const AkimaSpline spline_x(arc, std::move(x));
  const AkimaSpline spline_y(arc, std::move(y));

  Trajectory output;
  output.reserve(query.size());
  for (double s : query) {
    const auto upper = std::upper_bound(arc.begin(), arc.end(), s + 0.001);
    const std::size_t segment = std::min<std::size_t>(
        std::distance(arc.begin(), upper) - 1, path.size() - 1);
    const auto next = std::min(segment + 1, path.size() - 1);
    const double ratio = next == segment ? 0.0 :
        (s - arc[segment]) / (arc[next] - arc[segment]);
    auto point = path[segment];
    point.x = spline_x.value(s);
    point.y = spline_y.value(s);
    point.z += (path[next].z - point.z) * ratio;
    point.heading_rate_rps +=
        (path[next].heading_rate_rps - point.heading_rate_rps) * ratio;
    point.longitudinal_velocity_mps = path[segment].longitudinal_velocity_mps;
    output.push_back(point);
  }
  for (std::size_t i = 0; i + 1 < output.size(); ++i) {
    const auto &next = output[i + 1];
    auto &point = output[i];
    const double dx = next.x - point.x;
    const double dy = next.y - point.y;
    const double pitch = std::atan2(next.z - point.z, std::hypot(dx, dy));
    const double yaw = std::atan2(dy, dx);
    const double sp = std::sin(pitch / 2.0);
    const double cp = std::cos(pitch / 2.0);
    const double sy = std::sin(yaw / 2.0);
    const double cy = std::cos(yaw / 2.0);
    point.orientation_x = -sp * sy;
    point.orientation_y = sp * cy;
    point.orientation_z = cp * sy;
    point.orientation_w = cp * cy;
  }
  output.back().orientation_x = output[output.size() - 2].orientation_x;
  output.back().orientation_y = output[output.size() - 2].orientation_y;
  output.back().orientation_z = output[output.size() - 2].orientation_z;
  output.back().orientation_w = output[output.size() - 2].orientation_w;
  return output;
}

}  // 익명 네임스페이스

Trajectory postprocessBehaviorPath(
    const Trajectory &path, double forward_length_m, double interval_m) {
  if (!std::isfinite(forward_length_m) || !std::isfinite(interval_m) ||
      forward_length_m <= 0.0 || interval_m <= 0.0) {
    throw std::invalid_argument("경로 재보간 간격과 길이는 양수여야 합니다");
  }
  auto result = interpolate(filterClosePoints(path), forward_length_m, interval_m);
  bool found_stop = false;
  for (auto &point : result) {
    if (std::abs(point.longitudinal_velocity_mps) < 0.01F) found_stop = true;
    if (found_stop) point.longitudinal_velocity_mps = 0.0F;
  }
  return result;
}

}  // planning 네임스페이스
