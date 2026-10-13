// Copyright 2021 Tier IV, Inc.
// SPDX-License-Identifier: Apache-2.0
// JerkFilteredSmoother::apply의 QP 구성을 이관했습니다.

#pragma once

#include "planning/trajectory.hpp"

#include <Eigen/Core>

#include <cstddef>

namespace planning {

struct JerkQpParameters {
  double max_acceleration = 1.0;
  double min_deceleration = -1.0;
  double stop_deceleration = 0.0;
  double max_jerk = 1.0;
  double min_jerk = -1.0;
  double jerk_weight = 10.0;
  double over_velocity_weight = 100000.0;
  double over_acceleration_weight = 5000.0;
  double over_jerk_weight = 2000.0;
};

struct JerkQpProblem {
  std::size_t optimized_points = 0;
  Eigen::MatrixXd hessian;
  Eigen::MatrixXd constraints;
  Eigen::VectorXd linear_cost;
  Eigen::VectorXd lower_bound;
  Eigen::VectorXd upper_bound;
};

// 이미 재샘플링한 궤적을 입력받아 종점의 속도 0 조건을 포함한 원본 QP를
// 구성합니다. QP 풀이는 수행하지 않습니다.
JerkQpProblem buildJerkQpProblem(
    const Trajectory &resampled, double initial_velocity, double initial_acceleration,
    const JerkQpParameters &parameters);

// 풀이된 QP 벡터를 재샘플링된 궤적으로 변환합니다. 벡터는
// [속도 제곱, 가속도, 속도 여유 변수, 가속도 여유 변수, 저크 여유 변수]
// 순서이며 각 블록의 원소 수는 optimized_points입니다.
Trajectory convertJerkQpSolution(
    const Trajectory &resampled, const JerkQpProblem &problem,
    const Eigen::VectorXd &solution, double stop_deceleration);

}  // planning 네임스페이스
