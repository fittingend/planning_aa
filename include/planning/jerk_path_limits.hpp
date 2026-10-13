// Copyright 2021 Tier IV, Inc.
// SPDX-License-Identifier: Apache-2.0
// 스무더 입력 궤적의 곡률 기반 속도 제한입니다.

#pragma once

#include "planning/trajectory.hpp"

namespace planning {

struct JerkPathLimitParameters {
  double sample_interval = 0.1;
  double steering_input_interval = 1.0;
  double curvature_distance = 2.0;
  double max_lateral_acceleration = 1.0;
  double min_curve_velocity = 2.0;
  double deceleration_distance_before_curve = 3.5;
  double deceleration_distance_after_curve = 2.0;
  double min_jerk = -1.0;
  double max_acceleration = 1.0;
  double min_deceleration_for_lateral = -2.5;
  double wheelbase = 2.79;
  double max_steering_rate_degrees = 11.5;
  double curvature_threshold = 0.02;
};

// 원본의 조밀한 재표본화와 횡가속도 제한을 적용합니다.
Trajectory applyJerkLateralAccelerationLimit(
    const Trajectory &input, double initial_velocity,
    double initial_acceleration, const JerkPathLimitParameters &parameters);

// 횡가속도 제한 이후의 궤적에 조향 속도 제한을 적용합니다.
Trajectory applyJerkSteeringRateLimit(
    const Trajectory &input, const JerkPathLimitParameters &parameters);

}  // planning 네임스페이스
