// Copyright 2021 Tier IV, Inc.
// SPDX-License-Identifier: Apache-2.0
// JerkFiltered 스무더의 ROS 비의존 계산 경로입니다.

#pragma once

#include "planning/jerk_filter.hpp"
#include "planning/jerk_qp_solver.hpp"
#include "planning/jerk_resample.hpp"

#include <optional>

namespace planning {

struct JerkSmootherParameters {
  JerkFilterLimits filter{1.0, -1.0, 0.0, 1.0, -1.0};
  JerkResampleParameters resampling;
  JerkQpParameters qp;
  JerkQpSolveOptions solver;
};

struct JerkSmootherResult {
  std::optional<Trajectory> output;
  JerkFilterStages stages;
  Trajectory optimization_input;
  JerkQpSolveResult qp_result;
};

// 원본 apply의 필터, 재샘플링, QP, 결과 변환을 순서대로 실행합니다.
// QP가 수렴하지 않아도 원본 apply처럼 재표본화된 경로를 output에 둡니다.
JerkSmootherResult runJerkSmoother(
    const Trajectory &input, double initial_velocity, double initial_acceleration,
    const JerkSmootherParameters &parameters = {});

}  // planning 네임스페이스
