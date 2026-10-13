// Copyright 2023 TIER IV, Inc.
// SPDX-License-Identifier: Apache-2.0
// 경로 최적화의 비용 함수와 경계 제약 QP 입력입니다.

#pragma once

#include <Eigen/Core>

#include <cstddef>
#include <optional>
#include <vector>

namespace planning {

struct PathOptimizerCircle {
  double longitudinal_offset = 0.0;
  double radius = 0.0;
};

struct PathOptimizerBound {
  double lower = 0.0;
  double upper = 0.0;
};

struct PathOptimizerReference {
  double arc_length_to_next = 0.0;
  double curvature = 0.0;
  double alpha = 0.0;
  double avoidance_cost = 0.0;
  std::vector<double> beta;
  std::vector<PathOptimizerBound> circle_bounds;
  std::optional<Eigen::Vector2d> fixed_state;
};

struct PathOptimizerQpParameters {
  double wheelbase = 2.79;
  double vehicle_width = 1.896;
  double maximum_steering = 0.70;
  double optimization_center_offset = 2.79 * 0.8;
  double lateral_weight = 1.0;
  double yaw_weight = 0.0;
  double steering_weight = 1.0;
  double steering_rate_weight = 1.0;
  double terminal_lateral_weight = 100.0;
  double terminal_yaw_weight = 100.0;
  double goal_lateral_weight = 1000.0;
  double goal_yaw_weight = 1000.0;
  double avoidance_lateral_weight = 0.0;
  double avoidance_yaw_weight = 10.0;
  double avoidance_steering_weight = 100.0;
  double soft_collision_weight = 1.0;
  bool soft_constraints = true;
  bool hard_constraints = false;
  bool shared_slack = true;
  bool steering_limit_constraint = false;
};

struct PathOptimizerQpProblem {
  Eigen::MatrixXd hessian;
  Eigen::VectorXd gradient;
  Eigen::MatrixXd constraints;
  Eigen::VectorXd lower_bound;
  Eigen::VectorXd upper_bound;
  Eigen::Index state_variables = 0;
  Eigen::Index steering_variables = 0;
  Eigen::Index slack_variables = 0;
};

// 각 기준점의 beta와 경계는 circles와 같은 순서로 전달합니다.
// 최종점이 실제 목표점이면 goal_contained를 참으로 둡니다.
PathOptimizerQpProblem buildPathOptimizerQp(
    const std::vector<PathOptimizerReference> &references,
    const std::vector<PathOptimizerCircle> &circles,
    bool goal_contained,
    const PathOptimizerQpParameters &parameters = {});

// 원본의 fitting_uniform_circle 방식으로 차량 길이를 원 세 개에 배치합니다.
std::vector<PathOptimizerCircle> fittingUniformCircles(
    double vehicle_length, double vehicle_width, double rear_overhang,
    double left_overhang, double right_overhang, std::size_t count = 3);

}  // planning 네임스페이스
