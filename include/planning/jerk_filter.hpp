// Copyright 2021 Tier IV, Inc.
// SPDX-License-Identifier: Apache-2.0
// autoware_velocity_smoother의 필터 단계 로직을 이관했습니다.

#pragma once

#include "planning/trajectory.hpp"

namespace planning {

using FilterPoint = TrajectoryPoint;
using FilterTrajectory = Trajectory;

struct JerkFilterLimits {
  double max_acceleration;
  double min_deceleration;
  double stop_deceleration;
  double max_jerk;
  double min_jerk;
};

struct JerkFilterStages {
  FilterTrajectory forward;
  FilterTrajectory backward;
  FilterTrajectory merged;
};

// JerkFilteredSmoother::apply의 최적화 이전 필터입니다. 병합 결과는 최종
// 평활 궤적이 아니며 이후 재샘플링과 QP 최적화를 수행합니다.
JerkFilterStages runJerkFilterStage(
    const FilterTrajectory &input, double initial_velocity, double initial_acceleration,
    const JerkFilterLimits &limits);

}  // planning 네임스페이스
