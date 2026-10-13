// Copyright 2021, 2022 Tier IV, Inc.
// SPDX-License-Identifier: Apache-2.0
// autoware_planning_validator에서 궤적 자체 검사를 분리했습니다.

#pragma once

#include "planning/trajectory.hpp"

#include <cstddef>

namespace planning {

struct PrecheckThresholds {
  double max_interval_m = 100.0;
  double max_relative_angle_rad = 2.0;
  double max_curvature_per_m = 1.0;
  double max_lateral_acceleration_mps2 = 9.8;
  double max_longitudinal_acceleration_mps2 = 9.8;
  double min_longitudinal_acceleration_mps2 = -9.8;
  double max_steering_angle_rad = 1.414;
  double max_steering_rate_radps = 10.0;
  double wheelbase_m = 2.79;
  double max_velocity_deviation_mps = 100.0;
  double max_distance_deviation_m = 100.0;
  double max_longitudinal_distance_deviation_m = 1.0;
  double forward_length_deceleration_mps2 = -3.0;
  double forward_length_margin_m = 2.0;
};

struct EgoState {
  double x = 0.0;
  double y = 0.0;
  double longitudinal_velocity_mps = 0.0;
};

struct CheckResult {
  bool evaluated = false;
  bool passed = false;
  double observed = 0.0;
  std::size_t index = 0;
};

struct TrajectoryPrecheckResult {
  CheckResult size;
  CheckResult finite;
  CheckResult interval;
  CheckResult relative_angle;
  CheckResult curvature;
  CheckResult lateral_acceleration;
  CheckResult longitudinal_max_acceleration;
  CheckResult longitudinal_min_acceleration;
  CheckResult steering_angle;
  CheckResult steering_rate;

  // 궤적 자체 검사만 포함하며 차량 상태 검사 4개는 포함하지 않습니다.
  bool localChecksPass() const noexcept;
};

struct EgoCheckResult {
  CheckResult velocity_deviation;
  CheckResult distance_deviation;
  CheckResult longitudinal_distance_deviation;
  CheckResult forward_length;
  double forward_length_required_m = 0.0;

  bool checksPass() const noexcept;
};

struct TrajectoryValidationResult {
  TrajectoryPrecheckResult local;
  EgoCheckResult ego;
  bool allValid() const noexcept;
};

TrajectoryPrecheckResult checkTrajectoryLocal(
    const Trajectory &trajectory, const PrecheckThresholds &thresholds);

EgoCheckResult checkTrajectoryAgainstEgo(
    const Trajectory &trajectory, const EgoState &ego, const PrecheckThresholds &thresholds);

TrajectoryValidationResult validateTrajectory(
    const Trajectory &trajectory, const EgoState &ego, const PrecheckThresholds &thresholds);

}  // planning 네임스페이스
