// Copyright 2021 Tier IV, Inc.
// SPDX-License-Identifier: Apache-2.0
// autoware_velocity_smoother/src/resample.cpp와
// autoware_motion_utils/src/resample/resample.cpp를 바탕으로 작성했습니다.

#include "planning/jerk_resample.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

namespace planning {
namespace {

double lerp(double a, double b, double ratio) { return a + (b - a) * ratio; }

// 원본 motion_utils가 위치 좌표에 적용하는 Akima 보간 계수를 계산합니다.
struct AkimaSpline {
  std::vector<double> a;
  std::vector<double> b;
  std::vector<double> c;
  std::vector<double> d;

  AkimaSpline(const std::vector<double> &arcs, const std::vector<double> &values) {
    const std::size_t n = arcs.size();
    std::vector<double> slopes(n - 1);
    for (std::size_t i = 0; i + 1 < n; ++i) {
      slopes[i] = (values[i + 1] - values[i]) / (arcs[i + 1] - arcs[i]);
    }
    std::vector<double> derivatives(n);
    for (std::size_t i = 0; i < n; ++i) {
      if (i == 0) {
        derivatives[i] = slopes.front();
      } else if (i + 1 == n) {
        derivatives[i] = slopes.back();
      } else if (i == 1 || i + 2 == n) {
        derivatives[i] = (slopes[i - 1] + slopes[i]) / 2.0;
      } else {
        const double left = std::fabs(slopes[i + 1] - slopes[i]);
        const double right = std::fabs(slopes[i - 1] - slopes[i - 2]);
        const double sum = left + right;
        derivatives[i] = sum < 1.0e-5
            ? (slopes[i - 1] + slopes[i]) / 2.0
            : (left * slopes[i - 1] + right * slopes[i]) / sum;
      }
    }
    a.reserve(n - 1);
    b.reserve(n - 1);
    c.reserve(n - 1);
    d.reserve(n - 1);
    for (std::size_t i = 0; i + 1 < n; ++i) {
      const double h = arcs[i + 1] - arcs[i];
      a.push_back((derivatives[i] + derivatives[i + 1] - 2.0 * slopes[i]) / (h * h));
      b.push_back((3.0 * slopes[i] - 2.0 * derivatives[i] - derivatives[i + 1]) / h);
      c.push_back(derivatives[i]);
      d.push_back(values[i]);
    }
  }

  double evaluate(std::size_t segment, double distance) const {
    return d[segment] +
           (c[segment] + (b[segment] + a[segment] * distance) * distance) * distance;
  }
};

void appendArc(std::vector<double> &arcs, double value) {
  if (value - arcs.back() < 1.0e-3) {
    arcs.back() = value;
  } else {
    arcs.push_back(value);
  }
}

TrajectoryPoint interpolate(
    const TrajectoryPoint &a, const TrajectoryPoint &b, double ratio,
    float velocity, float acceleration) {
  TrajectoryPoint point;
  point.x = lerp(a.x, b.x, ratio);
  point.y = lerp(a.y, b.y, ratio);
  point.z = lerp(a.z, b.z, ratio);
  // 원본 스무더의 motion_utils 위치 보간 방식에 따라 단위 구면에서 방향을 보간합니다.
  double bx = b.orientation_x, by = b.orientation_y;
  double bz = b.orientation_z, bw = b.orientation_w;
  double dot = a.orientation_x * bx + a.orientation_y * by +
               a.orientation_z * bz + a.orientation_w * bw;
  if (dot < 0.0) {
    bx = -bx; by = -by; bz = -bz; bw = -bw;
    dot = -dot;
  }
  const double angle = std::acos(std::clamp(dot, -1.0, 1.0));
  double wa = 1.0 - ratio, wb = ratio;
  if (angle > 1.0e-6) {
    const double divisor = std::sin(angle);
    wa = std::sin((1.0 - ratio) * angle) / divisor;
    wb = std::sin(ratio * angle) / divisor;
  }
  point.orientation_x = wa * a.orientation_x + wb * bx;
  point.orientation_y = wa * a.orientation_y + wb * by;
  point.orientation_z = wa * a.orientation_z + wb * bz;
  point.orientation_w = wa * a.orientation_w + wb * bw;
  point.longitudinal_velocity_mps = velocity;
  point.acceleration_mps2 = acceleration;
  point.lateral_velocity_mps = static_cast<float>(lerp(a.lateral_velocity_mps, b.lateral_velocity_mps, ratio));
  point.heading_rate_rps = static_cast<float>(lerp(a.heading_rate_rps, b.heading_rate_rps, ratio));
  point.front_wheel_angle_rad = static_cast<float>(lerp(a.front_wheel_angle_rad, b.front_wheel_angle_rad, ratio));
  point.rear_wheel_angle_rad = static_cast<float>(lerp(a.rear_wheel_angle_rad, b.rear_wheel_angle_rad, ratio));
  return point;
}

std::vector<double> arcLengths(const Trajectory &input) {
  if (input.empty()) throw std::invalid_argument("재샘플링에는 궤적이 필요합니다");
  for (const auto &point : input) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.longitudinal_velocity_mps) ||
        !std::isfinite(point.acceleration_mps2)) {
      throw std::invalid_argument("재샘플링에는 유한한 궤적값이 필요합니다");
    }
  }
  std::vector<double> arcs{0.0};
  for (std::size_t i = 1; i < input.size(); ++i) {
    const double ds = std::hypot(input[i].x - input[i - 1].x,
                                 input[i].y - input[i - 1].y);
    if (!(ds > 0.0) || !std::isfinite(ds)) {
      throw std::invalid_argument("재샘플링에는 양수인 점 간격이 필요합니다");
    }
    arcs.push_back(arcs.back() + ds);
  }
  return arcs;
}

Trajectory sampleArcs(
    const Trajectory &input, const std::vector<double> &input_arcs,
    const std::vector<double> &output_arcs, bool endpoint_included,
    bool zero_order_hold, double stop_search_start_arc = 0.0,
    bool preserve_velocity_restarts = false) {
  if (input.size() < 2 || output_arcs.size() < 2 ||
      input_arcs.back() < output_arcs.back() ||
      !std::is_sorted(output_arcs.begin(), output_arcs.end())) return input;
  std::vector<double> x, y;
  x.reserve(input.size());
  y.reserve(input.size());
  for (const auto &point : input) {
    x.push_back(point.x);
    y.push_back(point.y);
  }
  const AkimaSpline x_spline(input_arcs, x);
  const AkimaSpline y_spline(input_arcs, y);
  double zero_arc = -1.0;
  for (std::size_t i = 0; i < input.size(); ++i) {
    if (input_arcs[i] >= stop_search_start_arc &&
        std::fabs(input[i].longitudinal_velocity_mps) < 1.0e-4) {
      zero_arc = input_arcs[i];
      break;
    }
  }
  Trajectory output;
  output.reserve(output_arcs.size() + (endpoint_included ? 1 : 0));
  for (double arc : output_arcs) {
    const auto upper = std::upper_bound(input_arcs.begin(), input_arcs.end(), arc);
    const std::size_t left = std::min(
        static_cast<std::size_t>(upper - input_arcs.begin() - 1), input.size() - 2);
    const auto hold_upper = std::upper_bound(
        input_arcs.begin(), input_arcs.end(), arc + 1.0e-3);
    const std::size_t hold = std::min(
        static_cast<std::size_t>(hold_upper - input_arcs.begin() - 1), input.size() - 1);
    const double ratio = (arc - input_arcs[left]) /
                         (input_arcs[left + 1] - input_arcs[left]);
    float velocity = zero_order_hold ? input[hold].longitudinal_velocity_mps :
        static_cast<float>(lerp(input[left].longitudinal_velocity_mps,
                                input[left + 1].longitudinal_velocity_mps, ratio));
    if (!preserve_velocity_restarts && zero_arc >= 0.0 && arc >= zero_arc) {
      velocity = 0.0F;
    }
    const float acceleration = zero_order_hold ? input[hold].acceleration_mps2 :
        static_cast<float>(lerp(input[left].acceleration_mps2,
                                input[left + 1].acceleration_mps2, ratio));
    auto point = interpolate(input[left], input[left + 1], ratio,
                             velocity, acceleration);
    point.x = x_spline.evaluate(left, arc - input_arcs[left]);
    point.y = y_spline.evaluate(left, arc - input_arcs[left]);
    output.push_back(point);
  }
  if (endpoint_included) {
    if (std::hypot(output.back().x - input.back().x,
                   output.back().y - input.back().y) < 1.0e-3) {
      output.back() = input.back();
    } else {
      output.push_back(input.back());
    }
  }
  return output;
}

double yaw(const TrajectoryPoint &point) {
  return std::atan2(
      2.0 * (point.orientation_w * point.orientation_z +
             point.orientation_x * point.orientation_y),
      1.0 - 2.0 * (point.orientation_y * point.orientation_y +
                   point.orientation_z * point.orientation_z));
}

double projectedArc(
    const Trajectory &input, const std::vector<double> &arcs,
    const TrajectoryPoint &pose, double distance_limit, double yaw_limit) {
  const auto nearest = [&](bool use_distance, bool use_yaw) {
    std::size_t best = 0;
    double best_distance = std::numeric_limits<double>::infinity();
    bool found = false;
    for (std::size_t i = 0; i < input.size(); ++i) {
      const double distance = std::hypot(input[i].x - pose.x, input[i].y - pose.y);
      const double yaw_delta = std::remainder(yaw(input[i]) - yaw(pose), 2.0 * 3.141592653589793);
      if ((use_distance && distance > distance_limit) ||
          (use_yaw && std::fabs(yaw_delta) > yaw_limit)) {
        if (found) break;
        continue;
      }
      if (distance < best_distance) {
        best = i;
        best_distance = distance;
        found = true;
      }
    }
    return std::pair<bool, std::size_t>{found, best};
  };
  auto match = nearest(true, true);
  if (!match.first) match = nearest(true, false);
  if (!match.first) match = nearest(false, false);
  const std::size_t index = match.second;
  std::size_t segment = index;
  if (index == input.size() - 1) segment = index - 1;
  else if (index > 0) {
    const double dx = input[index + 1].x - input[index].x;
    const double dy = input[index + 1].y - input[index].y;
    if (dx * (pose.x - input[index].x) +
        dy * (pose.y - input[index].y) <= 0.0) segment = index - 1;
  }
  const double dx = input[segment + 1].x - input[segment].x;
  const double dy = input[segment + 1].y - input[segment].y;
  const double length = arcs[segment + 1] - arcs[segment];
  const double offset = (dx * (pose.x - input[segment].x) +
                         dy * (pose.y - input[segment].y)) / length;
  return std::clamp(arcs[segment] + offset, 0.0, arcs.back());
}

}  // 익명 네임스페이스

Trajectory resampleJerkTrajectory(
    const Trajectory &input, double initial_velocity,
    const JerkResampleParameters &p) {
  if (input.empty()) {
    throw std::invalid_argument("resampling requires a trajectory");
  }
  const double values[] = {initial_velocity, p.max_trajectory_length,
                           p.min_trajectory_length, p.resample_time,
                           p.dense_resample_dt, p.dense_min_interval_distance,
                           p.sparse_resample_dt, p.sparse_min_interval_distance};
  for (double value : values) {
    if (!std::isfinite(value) || value < 0.0) {
      throw std::invalid_argument("resampling requires finite nonnegative parameters");
    }
  }
  if (p.dense_resample_dt == 0.0 || p.sparse_resample_dt == 0.0 ||
      p.dense_min_interval_distance == 0.0 || p.sparse_min_interval_distance == 0.0) {
    throw std::invalid_argument("resampling requires positive sampling intervals");
  }
  if (input.size() == 1) return input;

  std::vector<double> input_arcs{0.0};
  input_arcs.reserve(input.size());
  double stop_arc = -1.0;
  for (std::size_t i = 0; i < input.size(); ++i) {
    const auto &point = input[i];
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.longitudinal_velocity_mps) ||
        !std::isfinite(point.acceleration_mps2)) {
      throw std::invalid_argument("resampling requires finite trajectory points");
    }
    if (i != 0) {
      const double ds = std::hypot(point.x - input[i - 1].x, point.y - input[i - 1].y);
      if (!(ds > 0.0)) throw std::invalid_argument("resampling requires positive point spacing");
      input_arcs.push_back(input_arcs.back() + ds);
    }
    if (stop_arc < 0.0 && std::fabs(point.longitudinal_velocity_mps) < 1.0e-3) {
      stop_arc = input_arcs.back();
    }
  }

  const double length = input_arcs.back();
  const double nt = p.resample_time / std::max(p.dense_resample_dt, 0.001);
  const double dense_ds = std::max(initial_velocity * p.dense_resample_dt,
                                   p.dense_min_interval_distance);
  const double ns = p.min_trajectory_length / std::max(dense_ds, 0.001);
  const double count = std::max(nt, ns);
  std::vector<double> output_arcs{0.0};
  bool endpoint_included = false;
  bool stop_included = false;
  double distance = 0.0;
  for (std::size_t i = 1; static_cast<double>(i) <= count; ++i) {
    distance += i > nt ? std::max(p.sparse_min_interval_distance,
                                  p.sparse_resample_dt * initial_velocity) : dense_ds;
    if (distance > p.max_trajectory_length) break;
    if (distance >= length) {
      endpoint_included = true;
      break;
    }
    if (static_cast<double>(i) > nt && distance >= p.min_trajectory_length) {
      appendArc(output_arcs, p.min_trajectory_length);
      break;
    }
    if (stop_arc >= 0.0 && distance > stop_arc && !stop_included) {
      if (std::fabs(distance - stop_arc) > 1.0e-3) {
        appendArc(output_arcs, stop_arc);
      } else {
        distance = stop_arc;
      }
      stop_included = true;
    }
    appendArc(output_arcs, distance);
  }
  return sampleArcs(input, input_arcs, output_arcs, endpoint_included, true);
}

Trajectory resampleJerkNodeInput(
    const Trajectory &input, const TrajectoryPoint &current_pose,
    double nearest_distance, double nearest_yaw,
    const JerkResampleParameters &p, double nominal_ds) {
  if (!(nominal_ds > 0.0) || !std::isfinite(nominal_ds) ||
      !(p.max_trajectory_length > 0.0) || !(p.min_trajectory_length > 0.0) ||
      !std::isfinite(p.max_trajectory_length) ||
      !std::isfinite(p.min_trajectory_length) ||
      !std::isfinite(current_pose.x) || !std::isfinite(current_pose.y) ||
      !std::isfinite(nearest_distance) || !std::isfinite(nearest_yaw) ||
      nearest_distance < 0.0 || nearest_yaw < 0.0) {
    throw std::invalid_argument("잘못된 노드 입력 재샘플링 설정");
  }
  if (input.size() < 2) return input;
  const auto arcs = arcLengths(input);
  const double front = projectedArc(input, arcs, current_pose,
                                    nearest_distance, nearest_yaw);
  double stop_distance = std::min(p.max_trajectory_length, p.min_trajectory_length);
  for (std::size_t i = 0; i < input.size(); ++i) {
    if (arcs[i] >= front &&
        std::fabs(input[i].longitudinal_velocity_mps) < 1.0e-3) {
      stop_distance = std::min(arcs[i] - front, stop_distance);
      break;
    }
  }
  const double dense_start = std::max(stop_distance - 3.0, 0.0);
  std::vector<double> output_arcs{0.0};
  for (double arc = nominal_ds; arc < front; arc += nominal_ds) {
    appendArc(output_arcs, arc);
  }
  appendArc(output_arcs, front);

  double distance = 0.0;
  bool stop_included = false;
  bool endpoint_included = false;
  while (true) {
    double ds = nominal_ds;
    if (dense_start <= distance && distance <= stop_distance) ds = 0.01;
    distance += ds;
    if (distance > p.max_trajectory_length) break;
    if (distance + front >= arcs.back()) {
      endpoint_included = true;
      break;
    }
    if (distance >= p.min_trajectory_length) {
      appendArc(output_arcs, p.min_trajectory_length + front);
      break;
    }
    if (distance > stop_distance && !stop_included) {
      if (std::fabs(distance - stop_distance) > 1.0e-3) {
        appendArc(output_arcs, stop_distance + front);
      } else {
        distance = stop_distance;
      }
      stop_included = true;
    }
    appendArc(output_arcs, distance + front);
  }
  return sampleArcs(input, arcs, output_arcs, endpoint_included, true);
}

Trajectory resampleJerkUniform(const Trajectory &input, double interval) {
  if (!(interval > 0.0) || !std::isfinite(interval)) {
    throw std::invalid_argument("고정 간격 재표본화 설정이 잘못되었습니다");
  }
  if (input.size() < 2) return input;
  const auto arcs = arcLengths(input);
  std::vector<double> output_arcs;
  for (double arc = 0.0; arc < arcs.back(); arc += interval) {
    output_arcs.push_back(arc);
  }
  auto output = sampleArcs(input, arcs, output_arcs, false, true);
  output.back() = input.back();
  return output;
}

Trajectory resampleJerkAtArcs(
    const Trajectory &input, const std::vector<double> &output_arcs) {
  if (input.size() < 2) return input;
  if (output_arcs.size() < 2 || output_arcs.front() < 0.0 ||
      !std::all_of(output_arcs.begin(), output_arcs.end(),
                   [](double value) { return std::isfinite(value); }) ||
      std::adjacent_find(output_arcs.begin(), output_arcs.end(),
                         std::greater_equal<double>{}) != output_arcs.end()) {
    throw std::invalid_argument("재표본화 호길이 목록이 잘못되었습니다");
  }
  const auto input_arcs = arcLengths(input);
  if (output_arcs.back() > input_arcs.back()) {
    throw std::invalid_argument("재표본화 호길이가 입력 경로를 넘습니다");
  }
  return sampleArcs(input, input_arcs, output_arcs, false, true);
}

Trajectory resampleJerkNodeOutput(
    const Trajectory &input, double current_velocity,
    const TrajectoryPoint &current_pose, double nearest_distance,
    double nearest_yaw, const JerkResampleParameters &p,
    bool preserve_velocity_restarts) {
  if (!std::isfinite(current_velocity) ||
      !(p.dense_resample_dt > 0.0) || !(p.sparse_resample_dt > 0.0) ||
      !(p.dense_min_interval_distance > 0.0) ||
      !(p.sparse_min_interval_distance > 0.0) ||
      !(p.max_trajectory_length > 0.0) || !(p.min_trajectory_length > 0.0) ||
      !std::isfinite(p.dense_resample_dt) || !std::isfinite(p.sparse_resample_dt) ||
      !std::isfinite(p.dense_min_interval_distance) ||
      !std::isfinite(p.sparse_min_interval_distance) ||
      !std::isfinite(p.max_trajectory_length) ||
      !std::isfinite(p.min_trajectory_length) ||
      !std::isfinite(current_pose.x) || !std::isfinite(current_pose.y) ||
      !std::isfinite(nearest_distance) || !std::isfinite(nearest_yaw) ||
      nearest_distance < 0.0 || nearest_yaw < 0.0) {
    throw std::invalid_argument("잘못된 노드 출력 재샘플링 설정");
  }
  if (input.size() < 2) return input;
  const auto arcs = arcLengths(input);
  const double front = projectedArc(input, arcs, current_pose,
                                    nearest_distance, nearest_yaw);
  double stop_distance = -1.0;
  for (std::size_t i = 0; i < input.size(); ++i) {
    if (arcs[i] >= front &&
        std::fabs(input[i].longitudinal_velocity_mps) < 1.0e-3) {
      stop_distance = arcs[i] - front;
      break;
    }
  }
  std::vector<double> output_arcs{0.0};
  for (double arc = 0.1; arc < front; arc += 0.1) {
    appendArc(output_arcs, arc);
  }
  appendArc(output_arcs, front);

  const double nt = p.resample_time / std::max(p.dense_resample_dt, 0.001);
  const double dense_ds = std::max(current_velocity * p.dense_resample_dt,
                                   p.dense_min_interval_distance);
  const double ns = p.min_trajectory_length / std::max(dense_ds, 0.001);
  const double count = std::max(nt, ns);
  double distance = 0.0;
  bool stop_included = false;
  bool endpoint_included = false;
  for (std::size_t i = 1; static_cast<double>(i) <= count; ++i) {
    distance += i > nt ? std::max(p.sparse_min_interval_distance,
                                  p.sparse_resample_dt * current_velocity) : dense_ds;
    if (distance > p.max_trajectory_length) break;
    if (distance + front >= arcs.back()) {
      endpoint_included = true;
      break;
    }
    if (static_cast<double>(i) > nt && distance >= p.min_trajectory_length) {
      appendArc(output_arcs, p.min_trajectory_length + front);
      break;
    }
    if (stop_distance >= 0.0 && distance > stop_distance && !stop_included) {
      if (std::fabs(distance - stop_distance) > 1.0e-3) {
        appendArc(output_arcs, stop_distance + front);
      } else {
        distance = stop_distance;
      }
      stop_included = true;
    }
    appendArc(output_arcs, distance + front);
  }
  return sampleArcs(input, arcs, output_arcs, endpoint_included, false, front,
                    preserve_velocity_restarts);
}

}  // planning 네임스페이스
