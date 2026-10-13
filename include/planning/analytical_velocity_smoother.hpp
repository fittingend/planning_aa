// SPDX-License-Identifier: Apache-2.0
// 행동 속도 계획의 Analytical 스무더 앞쪽 속도 계산을 분리합니다.

#pragma once

#include "planning/trajectory.hpp"

#include <cstddef>

namespace planning {

struct AnalyticalForwardParameters {
  double max_acceleration_mps2 = 1.0;
  double min_acceleration_mps2 = -1.0;
  double max_jerk_mps3 = 0.3;
  double min_jerk_mps3 = -0.3;
  double feedback_gain = 0.3;
};

struct AnalyticalBackwardParameters {
  double start_jerk_mps3 = -0.1;
  double minimum_mild_jerk_mps3 = -0.3;
  double minimum_jerk_mps3 = -1.5;
  double minimum_mild_acceleration_mps2 = -1.0;
  double minimum_acceleration_mps2 = -2.5;
  double jerk_step_mps3 = -0.01;
};

// 원본의 곡률 5m 기준과 전후 감속 범위로 속도 상한을 만듭니다.
Trajectory applyAnalyticalLateralAccelerationLimit(
    const Trajectory &input,
    double sample_interval_m = 0.1,
    double maximum_lateral_acceleration_mps2 = 1.0,
    double minimum_curve_velocity_mps = 2.0,
    double before_curve_m = 3.5,
    double after_curve_m = 2.0);

// 원본의 전방 피드백 저크 제한을 적용합니다. 감속 목표 역방향 계산은 별도입니다.
Trajectory applyAnalyticalForwardJerkFilter(
    const Trajectory &reference, std::size_t start_index,
    double initial_velocity_mps, double initial_acceleration_mps2,
    const AnalyticalForwardParameters &parameters = {});

// 경로의 감속 목표를 찾고 전방·역방향 제한을 연결합니다.
Trajectory smoothAnalyticalVelocity(
    const Trajectory &reference, double initial_velocity_mps,
    double initial_acceleration_mps2,
    const AnalyticalForwardParameters &forward = {},
    const AnalyticalBackwardParameters &backward = {});

}  // planning 네임스페이스
