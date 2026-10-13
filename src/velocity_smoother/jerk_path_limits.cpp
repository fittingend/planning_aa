// Copyright 2021 Tier IV, Inc.
// SPDX-License-Identifier: Apache-2.0
// 원본 SmootherBase의 곡률 기반 속도 제한을 이관했습니다.

#include "planning/jerk_path_limits.hpp"

#include "planning/jerk_resample.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace planning {
namespace {

double distance(const TrajectoryPoint &a, const TrajectoryPoint &b) {
  return std::hypot(a.x - b.x, a.y - b.y);
}

double curvature(
    const TrajectoryPoint &a, const TrajectoryPoint &b,
    const TrajectoryPoint &c) {
  const double ab = distance(a, b);
  const double bc = distance(b, c);
  const double ca = distance(c, a);
  const double denominator = ab * bc * ca;
  if (denominator < 1.0e-9) return 0.0;
  const double cross =
      (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
  return 2.0 * cross / denominator;
}

std::vector<double> curvatures(
    const Trajectory &trajectory, std::size_t index_distance) {
  std::vector<double> values(trajectory.size(), 0.0);
  if (trajectory.size() < 3) return values;
  index_distance = std::clamp(
      index_distance, std::size_t{1}, (trajectory.size() - 1) / 2);
  for (std::size_t i = 1; i + 1 < trajectory.size(); ++i) {
    const std::size_t back = i - std::min(index_distance, i);
    const std::size_t front = i + std::min(index_distance, trajectory.size() - 1 - i);
    values[i] = curvature(trajectory[back], trajectory[i], trajectory[front]);
  }
  values.front() = values[1];
  values.back() = values[values.size() - 2];
  return values;
}

void validate(const JerkPathLimitParameters &p) {
  const double values[] = {
      p.sample_interval, p.steering_input_interval, p.curvature_distance,
      p.max_lateral_acceleration,
      p.min_curve_velocity, p.deceleration_distance_before_curve,
      p.deceleration_distance_after_curve, p.min_jerk, p.max_acceleration,
      p.min_deceleration_for_lateral, p.wheelbase,
      p.max_steering_rate_degrees, p.curvature_threshold};
  for (double value : values) {
    if (!std::isfinite(value)) {
      throw std::invalid_argument("곡률 속도 제한에는 유한한 설정이 필요합니다");
    }
  }
  if (p.sample_interval <= 0.0 || p.steering_input_interval <= 0.0 ||
      p.curvature_distance <= 0.0 ||
      p.max_lateral_acceleration <= 0.0 || p.wheelbase <= 0.0 ||
      p.max_steering_rate_degrees <= 0.0) {
    throw std::invalid_argument("곡률 속도 제한 설정이 잘못되었습니다");
  }
}

}  // 익명 네임스페이스

Trajectory applyJerkLateralAccelerationLimit(
    const Trajectory &input, double initial_velocity,
    double initial_acceleration, const JerkPathLimitParameters &p) {
  validate(p);
  if (input.size() < 3) return input;
  auto output = resampleJerkUniform(input, p.sample_interval);
  const auto index_distance = static_cast<std::size_t>(
      std::max(static_cast<int>(p.curvature_distance / p.sample_interval), 1));
  const auto bends = curvatures(output, index_distance);
  const auto before = static_cast<std::size_t>(
      std::round(p.deceleration_distance_before_curve / p.sample_interval));
  const auto after = static_cast<std::size_t>(
      std::round(p.deceleration_distance_after_curve / p.sample_interval));

  std::vector<double> smooth_velocity(output.size());
  smooth_velocity.front() = initial_velocity;
  double acceleration = initial_acceleration;
  for (std::size_t i = 1; i < output.size(); ++i) {
    const double dt = distance(output[i - 1], output[i]) /
                      std::max(smooth_velocity[i - 1], 1.0e-5);
    smooth_velocity[i] = smooth_velocity[i - 1] + acceleration * dt +
                         0.5 * p.min_jerk * dt * dt;
    acceleration = std::clamp(
        acceleration + p.min_jerk * dt,
        p.min_deceleration_for_lateral, p.max_acceleration);
  }

  for (std::size_t i = 0; i < output.size(); ++i) {
    double bend = 0.0;
    const std::size_t start = i > after ? i - after : 0;
    const std::size_t end = std::min(output.size(), i + before + 1);
    for (std::size_t j = start; j < end; ++j) {
      bend = std::max(bend, std::fabs(bends[j]));
    }
    double limit = std::sqrt(
        std::fabs(p.max_lateral_acceleration) / std::max(bend, 1.0e-5));
    limit = std::max({limit, p.min_curve_velocity, smooth_velocity[i]});
    if (output[i].longitudinal_velocity_mps > limit) {
      output[i].longitudinal_velocity_mps = static_cast<float>(limit);
    }
  }
  return output;
}

Trajectory applyJerkSteeringRateLimit(
    const Trajectory &input, const JerkPathLimitParameters &p) {
  validate(p);
  if (input.size() < 3) return input;
  auto output = input;
  const auto index_distance = static_cast<std::size_t>(
      std::max(static_cast<int>(p.curvature_distance / p.steering_input_interval), 1));
  const auto bends = curvatures(output, index_distance);
  std::vector<double> steering_rate(output.size(), 0.0);
  for (std::size_t i = 0; i + 1 < output.size(); ++i) {
    const double mean_velocity =
        0.5 * (output[i].longitudinal_velocity_mps +
               output[i + 1].longitudinal_velocity_mps);
    output[i + 1].front_wheel_angle_rad = static_cast<float>(
        std::atan(p.wheelbase * bends[i + 1]));
    output[i].front_wheel_angle_rad = static_cast<float>(
        std::atan(p.wheelbase * bends[i]));
    const double dt = std::max(
        p.steering_input_interval / mean_velocity,
        std::numeric_limits<double>::epsilon());
    steering_rate[i] = std::fabs(
        output[i + 1].front_wheel_angle_rad -
        output[i].front_wheel_angle_rad) / dt;
  }
  steering_rate.back() = steering_rate[steering_rate.size() - 2];
  for (std::size_t i = 1; i + 1 < steering_rate.size(); ++i) {
    steering_rate[i] = (
        steering_rate[i - 1] + steering_rate[i] + steering_rate[i + 1]) / 3.0;
  }
  const double maximum_rate =
      p.max_steering_rate_degrees * 3.141592653589793 / 180.0;
  for (std::size_t i = 0; i + 1 < output.size(); ++i) {
    if (std::fabs(bends[i]) < p.curvature_threshold ||
        steering_rate[i] < maximum_rate) {
      continue;
    }
    const double mean_velocity =
        (output[i].longitudinal_velocity_mps +
         output[i + 1].longitudinal_velocity_mps) / 2.0;
    const double target_mean_velocity =
        mean_velocity * maximum_rate / steering_rate[i];
    for (std::size_t j = i; j <= i + 1; ++j) {
      auto &velocity = output[j].longitudinal_velocity_mps;
      const float target = static_cast<float>(std::max(
          p.min_curve_velocity,
          std::min(target_mean_velocity,
                   velocity * target_mean_velocity / mean_velocity)));
      velocity = std::min(velocity, target);
    }
  }
  return output;
}

}  // planning 네임스페이스
