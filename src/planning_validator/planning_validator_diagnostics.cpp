// Copyright 2021, 2022 Tier IV, Inc.
// SPDX-License-Identifier: Apache-2.0
// 원본 validator의 진단 이름과 오류 횟수 정책을 ROS 없이 계산합니다.

#include "planning/planning_validator_diagnostics.hpp"

#include <array>
#include <utility>

namespace planning {

std::vector<ValidationDiagnostic> makeValidationDiagnostics(
    const TrajectoryValidationResult &checks,
    std::size_t consecutive_invalid_count,
    std::size_t error_count_threshold) {
  struct Source {
    const char *name;
    const char *failure_message;
    bool passed;
  };
  const std::array<Source, 14> sources{{
      {"trajectory_validation_size", "invalid trajectory size is found", checks.local.size.passed},
      {"trajectory_validation_finite", "infinite value is found", checks.local.finite.passed},
      {"trajectory_validation_interval", "points interval is too long", checks.local.interval.passed},
      {"trajectory_validation_relative_angle", "relative angle is too large", checks.local.relative_angle.passed},
      {"trajectory_validation_curvature", "curvature is too large", checks.local.curvature.passed},
      {"trajectory_validation_lateral_acceleration", "lateral acceleration is too large", checks.local.lateral_acceleration.passed},
      {"trajectory_validation_acceleration", "acceleration is too large", checks.local.longitudinal_max_acceleration.passed},
      {"trajectory_validation_deceleration", "deceleration is too large", checks.local.longitudinal_min_acceleration.passed},
      {"trajectory_validation_steering", "expected steering is too large", checks.local.steering_angle.passed},
      {"trajectory_validation_steering_rate", "expected steering rate is too large", checks.local.steering_rate.passed},
      {"trajectory_validation_velocity_deviation", "velocity deviation is too large", checks.ego.velocity_deviation.passed},
      {"trajectory_validation_distance_deviation", "distance deviation is too large", checks.ego.distance_deviation.passed},
      {"trajectory_validation_longitudinal_distance_deviation", "longitudinal distance deviation is too large", checks.ego.longitudinal_distance_deviation.passed},
      {"trajectory_validation_forward_trajectory_length", "trajectory length is too short", checks.ego.forward_length.passed},
  }};
  std::vector<ValidationDiagnostic> output;
  output.reserve(sources.size());
  for (const auto &source : sources) {
    ValidationDiagnostic item;
    item.name = source.name;
    if (source.passed) {
      item.message = "validated.";
    } else if (consecutive_invalid_count < error_count_threshold) {
      item.level = ValidationDiagnosticLevel::Warn;
      item.message = std::string(source.failure_message) +
          " (invalid count is less than error threshold: " +
          std::to_string(consecutive_invalid_count) + " < " +
          std::to_string(error_count_threshold) + ")";
    } else {
      item.level = ValidationDiagnosticLevel::Error;
      item.message = source.failure_message;
    }
    output.push_back(std::move(item));
  }
  return output;
}

}  // planning 네임스페이스
