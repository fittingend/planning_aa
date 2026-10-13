// Copyright 2023 TIER IV, Inc.
// SPDX-License-Identifier: Apache-2.0
// 원본 path_optimizer의 자전거 모델과 상태 방정식 행렬을 이관했습니다.

#include "planning/path_optimizer_model.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace planning {

BicycleStateStep bicycleStateStep(
    const double wheelbase, const double steering_limit,
    const double curvature, const double arc_length) {
  if (!std::isfinite(wheelbase) || wheelbase <= 0.0 ||
      !std::isfinite(steering_limit) || steering_limit <= 0.0 ||
      !std::isfinite(curvature) || !std::isfinite(arc_length) ||
      arc_length < 0.0) {
    throw std::invalid_argument("자전거 운동 모델의 입력이 유효하지 않습니다");
  }
  const double reference_steering = std::atan(wheelbase * curvature);
  const double limited_steering =
      std::clamp(reference_steering, -steering_limit, steering_limit);
  BicycleStateStep step;
  step.transition << 1.0, arc_length, 0.0, 1.0;
  step.steering << 0.0,
      arc_length / wheelbase / std::pow(std::cos(reference_steering), 2.0);
  step.offset << 0.0,
      -arc_length * curvature + arc_length / wheelbase *
          (std::tan(limited_steering) - limited_steering /
           std::pow(std::cos(limited_steering), 2.0));
  return step;
}

PathOptimizerStateEquation buildPathOptimizerStateEquation(
    const std::vector<double> &arc_lengths, const double wheelbase,
    const double steering_limit) {
  const Eigen::Index point_count =
      static_cast<Eigen::Index>(arc_lengths.size()) + 1;
  PathOptimizerStateEquation equation;
  equation.transition = Eigen::MatrixXd::Zero(2 * point_count, 2 * point_count);
  equation.steering = Eigen::MatrixXd::Zero(2 * point_count, point_count - 1);
  equation.offset = Eigen::VectorXd::Zero(2 * point_count);
  equation.transition.block<2, 2>(0, 0).setIdentity();
  for (Eigen::Index i = 1; i < point_count; ++i) {
    const auto step = bicycleStateStep(
        wheelbase, steering_limit, 0.0,
        arc_lengths.at(static_cast<std::size_t>(i - 1)));
    equation.transition.block<2, 2>(2 * i, 2 * (i - 1)) = step.transition;
    equation.steering.block<2, 1>(2 * i, i - 1) = step.steering;
    equation.offset.segment<2>(2 * i) = step.offset;
  }
  return equation;
}

}  // planning 네임스페이스
