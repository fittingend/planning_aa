// Copyright 2023 TIER IV, Inc.
// SPDX-License-Identifier: Apache-2.0
// 경로 최적화에서 사용하는 자전거 운동 모델의 상태 방정식입니다.

#pragma once

#include <Eigen/Core>

#include <vector>

namespace planning {

struct BicycleStateStep {
  Eigen::Matrix2d transition;
  Eigen::Vector2d steering;
  Eigen::Vector2d offset;
};

// 상태는 [횡방향 오차, 방향 오차], 입력은 조향각입니다.
BicycleStateStep bicycleStateStep(
    double wheelbase, double steering_limit, double curvature,
    double arc_length);

struct PathOptimizerStateEquation {
  Eigen::MatrixXd transition;
  Eigen::MatrixXd steering;
  Eigen::VectorXd offset;
};

// 원본 경로 최적화는 기준점의 곡률을 상태 방정식에 적용하지 않습니다.
// arc_lengths에는 각 기준점에서 다음 기준점까지의 거리를 순서대로 넣습니다.
PathOptimizerStateEquation buildPathOptimizerStateEquation(
    const std::vector<double> &arc_lengths, double wheelbase,
    double steering_limit);

}  // planning 네임스페이스
