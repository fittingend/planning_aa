// Copyright 2022 TIER IV, Inc.
// SPDX-License-Identifier: Apache-2.0
// 순항 대상 선택과 기본 PID 속도 제한 계산입니다.

#include "planning/obstacle_cruise_pid_core.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace planning {
namespace {
double lowpass(std::optional<double> &previous, double input, double gain) {
  const double output = previous ? gain * *previous + (1.0 - gain) * input
                                 : input;
  previous = output;
  return output;
}
}  // 익명 네임스페이스

ObstacleCruisePidCore::ObstacleCruisePidCore(CruisePidParameters parameters)
    : parameters_(parameters) {
  const auto valid_gain = [](double gain) {
    return std::isfinite(gain) && gain >= 0.0 && gain <= 1.0;
  };
  if (!valid_gain(parameters.normalized_error_filter_gain) ||
      !valid_gain(parameters.measurement_filter_gain) ||
      !valid_gain(parameters.output_filter_gain) ||
      parameters.minimum_ego_acceleration_for_rss >= 0.0 ||
      parameters.minimum_object_acceleration_for_rss >= 0.0 ||
      !std::isfinite(parameters.safe_distance_margin) ||
      parameters.safe_distance_margin < 0.0 ||
      !std::isfinite(parameters.kp) || !std::isfinite(parameters.ki) ||
      !std::isfinite(parameters.kd))
    throw std::invalid_argument("순항 PID 설정이 유효하지 않습니다");
}

void ObstacleCruisePidCore::reset() {
  filtered_distance_.reset();
  filtered_obstacle_velocity_.reset();
  filtered_error_distance_.reset();
  filtered_normalized_error_.reset();
  filtered_output_velocity_.reset();
  previous_pid_error_.reset();
  pid_error_sum_ = 0.0;
}

std::optional<CruisePidVelocityLimit> ObstacleCruisePidCore::update(
    const double ego_velocity,
    const std::vector<CruisePidObstacle> &obstacles) {
  if (!std::isfinite(ego_velocity))
    throw std::invalid_argument("순항 자차 속도가 유효하지 않습니다");
  if (obstacles.empty()) {
    filtered_distance_.reset();
    filtered_obstacle_velocity_.reset();
    filtered_error_distance_.reset();
    filtered_normalized_error_.reset();
    filtered_output_velocity_.reset();
    // 원본 PID 계산기의 누적 오차와 직전 오차는 대상이 사라져도 유지합니다.
    return std::nullopt;
  }
  std::optional<CruisePidObstacle> chosen;
  double chosen_distance = 0.0;
  double chosen_target = 0.0;
  double chosen_error = 0.0;
  for (const auto &obstacle : obstacles) {
    if (!std::isfinite(obstacle.distance_from_ego_front) ||
        !std::isfinite(obstacle.velocity))
      throw std::invalid_argument("순항 객체 입력이 유효하지 않습니다");
    const double distance = obstacle.distance_from_ego_front +
        (obstacle.velocity - ego_velocity) * parameters_.evaluation_time;
    const double target =
        ego_velocity * parameters_.idling_time +
        ego_velocity * ego_velocity * 0.5 /
            std::abs(parameters_.minimum_ego_acceleration_for_rss) -
        obstacle.velocity * obstacle.velocity * 0.5 /
            std::abs(parameters_.minimum_object_acceleration_for_rss) +
        parameters_.safe_distance_margin;
    const double error = distance - target;
    if (chosen && error > chosen_error) continue;
    chosen = obstacle;
    chosen_distance = distance;
    chosen_target = target;
    chosen_error = error;
  }
  if (!chosen) return std::nullopt;
  const double filtered_distance = lowpass(
      filtered_distance_, chosen_distance,
      parameters_.measurement_filter_gain);
  lowpass(filtered_obstacle_velocity_, chosen->velocity,
          parameters_.measurement_filter_gain);
  const double filtered_error = lowpass(
      filtered_error_distance_, chosen_error,
      parameters_.measurement_filter_gain);
  if (std::abs(filtered_distance) < 1e-9)
    throw std::domain_error("순항 정규화 거리의 분모가 0입니다");
  const double normalized_error = lowpass(
      filtered_normalized_error_, filtered_error / filtered_distance,
      parameters_.normalized_error_filter_gain);
  const double pid_error = parameters_.quadratic_error
      ? std::copysign(normalized_error * normalized_error, normalized_error)
      : normalized_error;
  pid_error_sum_ += pid_error;
  const double derivative = previous_pid_error_
      ? pid_error - *previous_pid_error_ : 0.0;
  previous_pid_error_ = pid_error;
  const double pid_velocity = parameters_.kp * pid_error +
      parameters_.ki * pid_error_sum_ + parameters_.kd * derivative;
  const double additional_velocity = pid_error > 0.0
      ? pid_velocity * parameters_.acceleration_output_ratio
      : pid_velocity;
  const double positive_target = std::max(
      parameters_.minimum_target_velocity,
      ego_velocity + additional_velocity);
  const double output_velocity = lowpass(
      filtered_output_velocity_, positive_target,
      parameters_.output_filter_gain);
  return CruisePidVelocityLimit{
      output_velocity, parameters_.minimum_cruise_acceleration,
      filtered_distance, chosen_target, filtered_error};
}

}  // planning 네임스페이스
