// Copyright 2023 TIER IV, Inc.
// SPDX-License-Identifier: Apache-2.0
// 횡단보도 정지 판단에 쓰이는 저크·가속도 제한 감속 거리를 계산합니다.

#include "planning/crosswalk_core.hpp"

#include <algorithm>
#include <cmath>
#include <optional>
#include <stdexcept>

namespace planning {
namespace {

struct Motion {
  double distance;
  double velocity;
  double acceleration;
};

Motion update(Motion initial, double jerk, double duration) {
  return {
      initial.distance + initial.velocity * duration +
          0.5 * initial.acceleration * duration * duration +
          jerk * duration * duration * duration / 6.0,
      initial.velocity + initial.acceleration * duration +
          0.5 * jerk * duration * duration,
      initial.acceleration + jerk * duration};
}

std::optional<double> checkedDistance(Motion end, double target_velocity) {
  if (!std::isfinite(end.distance) ||
      std::abs(end.velocity - target_velocity) > 0.3 ||
      std::abs(end.acceleration) > 0.1) return std::nullopt;
  return end.distance;
}

}  // 익명 네임스페이스

std::optional<double> calcCrosswalkDecelDistance(
    double current_velocity_mps, double target_velocity_mps,
    double current_acceleration_mps2, double min_acceleration_mps2,
    double recovery_jerk_mps3, double braking_jerk_mps3) {
  if (!std::isfinite(current_velocity_mps) ||
      !std::isfinite(target_velocity_mps) ||
      !std::isfinite(current_acceleration_mps2) ||
      !std::isfinite(min_acceleration_mps2) ||
      !std::isfinite(recovery_jerk_mps3) ||
      !std::isfinite(braking_jerk_mps3) ||
      min_acceleration_mps2 >= 0.0 || recovery_jerk_mps3 <= 0.0 ||
      braking_jerk_mps3 >= 0.0) {
    throw std::invalid_argument("감속 거리의 가속도·저크 설정이 잘못되었습니다");
  }
  if (current_velocity_mps < target_velocity_mps) return std::nullopt;
  constexpr double epsilon = 1.0e-3;
  const double jerk_before = min_acceleration_mps2 < current_acceleration_mps2
      ? braking_jerk_mps3 : recovery_jerk_mps3;
  const double before_time =
      (min_acceleration_mps2 - current_acceleration_mps2) / jerk_before;
  const double recovery_time = -min_acceleration_mps2 / recovery_jerk_mps3;
  const double constant_time =
      (target_velocity_mps - current_velocity_mps -
       current_acceleration_mps2 * before_time -
       0.5 * jerk_before * before_time * before_time -
       min_acceleration_mps2 * recovery_time -
       0.5 * recovery_jerk_mps3 * recovery_time * recovery_time) /
      min_acceleration_mps2;
  const bool needs_decel =
      0.5 * (-current_acceleration_mps2) / recovery_jerk_mps3 *
          current_acceleration_mps2 >
      target_velocity_mps - current_velocity_mps;
  const Motion initial{0.0, current_velocity_mps, current_acceleration_mps2};
  if (constant_time > epsilon) {
    const double first_time = (min_acceleration_mps2 - current_acceleration_mps2) /
                              jerk_before;
    const auto first = update(initial, jerk_before,
                              first_time > epsilon ? first_time : 0.0);
    const auto middle = update(first, 0.0, constant_time);
    const auto end = update(middle, recovery_jerk_mps3,
                            recovery_time > epsilon ? recovery_time : 0.0);
    return checkedDistance(end, target_velocity_mps);
  }
  if (needs_decel || current_acceleration_mps2 > epsilon) {
    const double peak_squared =
        (target_velocity_mps - current_velocity_mps -
         0.5 * (-current_acceleration_mps2) / braking_jerk_mps3 *
             current_acceleration_mps2) *
        (2.0 * recovery_jerk_mps3 * braking_jerk_mps3 /
         (recovery_jerk_mps3 - braking_jerk_mps3));
    if (peak_squared < 0.0) return std::nullopt;
    const double peak_acceleration = -std::sqrt(peak_squared);
    const double first_time =
        (peak_acceleration - current_acceleration_mps2) / braking_jerk_mps3;
    const auto first = update(initial, braking_jerk_mps3,
                              first_time > epsilon ? first_time : 0.0);
    const double last_time = -peak_acceleration / recovery_jerk_mps3;
    const auto end = update(
        {first.distance, first.velocity, peak_acceleration},
        recovery_jerk_mps3, last_time > epsilon ? last_time : 0.0);
    return checkedDistance(end, target_velocity_mps);
  }
  const double recovery_only_time =
      -current_acceleration_mps2 / recovery_jerk_mps3;
  const auto end = update(initial, recovery_jerk_mps3,
                          recovery_only_time > epsilon ? recovery_only_time : 0.0);
  return checkedDistance(end, target_velocity_mps);
}

CrosswalkResult planCrosswalkObjectStopWithDynamics(
    const Trajectory &path, const std::vector<CrosswalkPoint> &polygon,
    CrosswalkPoint ego, const CrosswalkCollisionPoint &collision,
    double ego_velocity_mps, double ego_acceleration_mps2,
    CrosswalkParameters crosswalk_parameters,
    CrosswalkObjectStopParameters object_parameters,
    CrosswalkBrakingParameters braking_parameters) {
  const double velocity = std::max(0.0, ego_velocity_mps);
  const auto weak = calcCrosswalkDecelDistance(
      velocity, 0.0, ego_acceleration_mps2,
      braking_parameters.preferred_min_acc_mps2,
      braking_parameters.recovery_jerk_mps3,
      braking_parameters.preferred_min_jerk_mps3);
  if (!weak) {
    CrosswalkResult result;
    result.trajectory = path;
    return result;
  }
  const auto strong = calcCrosswalkDecelDistance(
      velocity, 0.0, ego_acceleration_mps2,
      braking_parameters.strong_min_acc_mps2,
      braking_parameters.recovery_jerk_mps3,
      braking_parameters.strong_min_jerk_mps3);
  return planCrosswalkObjectStop(
      path, polygon, ego, collision, *weak, strong.value_or(0.0),
      crosswalk_parameters, object_parameters);
}

}  // planning 네임스페이스
