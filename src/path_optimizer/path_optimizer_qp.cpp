// Copyright 2023 TIER IV, Inc.
// SPDX-License-Identifier: Apache-2.0
// 원본 MPTOptimizer의 기본 QP 비용과 제약 행렬을 구성합니다.

#include "planning/path_optimizer_qp.hpp"

#include "planning/path_optimizer_model.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace planning {
namespace {

constexpr double kInfinity = 1.0e20;

double blend(double base, double avoidance, double amount) {
  return base + (avoidance - base) * amount;
}

void validate(
    const std::vector<PathOptimizerReference> &references,
    const std::vector<PathOptimizerCircle> &circles,
    const PathOptimizerQpParameters &p) {
  if (references.size() < 2 || circles.empty() ||
      !std::isfinite(p.wheelbase) || p.wheelbase <= 0.0 ||
      !std::isfinite(p.vehicle_width) || p.vehicle_width <= 0.0 ||
      !std::isfinite(p.maximum_steering) || p.maximum_steering <= 0.0 ||
      !std::isfinite(p.optimization_center_offset) ||
      !std::isfinite(p.soft_collision_weight) ||
      p.soft_collision_weight < 0.0) {
    throw std::invalid_argument("경로 최적화 QP의 기본 입력이 유효하지 않습니다");
  }
  const double weights[] = {
      p.lateral_weight, p.yaw_weight, p.steering_weight,
      p.steering_rate_weight, p.terminal_lateral_weight,
      p.terminal_yaw_weight, p.goal_lateral_weight, p.goal_yaw_weight,
      p.avoidance_lateral_weight, p.avoidance_yaw_weight,
      p.avoidance_steering_weight};
  for (const double weight : weights) {
    if (!std::isfinite(weight) || weight < 0.0) {
      throw std::invalid_argument("경로 최적화 QP의 가중치가 유효하지 않습니다");
    }
  }
  for (const auto &circle : circles) {
    if (!std::isfinite(circle.longitudinal_offset) ||
        !std::isfinite(circle.radius) || circle.radius <= 0.0) {
      throw std::invalid_argument("차량 원 정보가 유효하지 않습니다");
    }
  }
  for (const auto &reference : references) {
    if (!std::isfinite(reference.arc_length_to_next) ||
        reference.arc_length_to_next < 0.0 ||
        !std::isfinite(reference.curvature) ||
        !std::isfinite(reference.alpha) ||
        !std::isfinite(reference.avoidance_cost) ||
        reference.avoidance_cost < 0.0 || reference.avoidance_cost > 1.0 ||
        reference.beta.size() != circles.size() ||
        reference.circle_bounds.size() != circles.size() ||
        (reference.fixed_state && !reference.fixed_state->allFinite())) {
      throw std::invalid_argument("경로 최적화 기준점이 유효하지 않습니다");
    }
    for (std::size_t i = 0; i < circles.size(); ++i) {
      const auto &bound = reference.circle_bounds[i];
      if (!std::isfinite(reference.beta[i]) ||
          !std::isfinite(bound.lower) || !std::isfinite(bound.upper) ||
          bound.lower > bound.upper) {
        throw std::invalid_argument("경로 최적화 경계가 유효하지 않습니다");
      }
    }
  }
}

}  // 익명 네임스페이스

std::vector<PathOptimizerCircle> fittingUniformCircles(
    const double vehicle_length, const double vehicle_width,
    const double rear_overhang, const double left_overhang,
    const double right_overhang, const std::size_t count) {
  if (count < 2 || !std::isfinite(vehicle_length) || vehicle_length <= 0.0 ||
      !std::isfinite(vehicle_width) || vehicle_width <= 0.0 ||
      !std::isfinite(rear_overhang) || !std::isfinite(left_overhang) ||
      !std::isfinite(right_overhang)) {
    throw std::invalid_argument("차량 크기가 유효하지 않습니다");
  }
  const double radius =
      vehicle_width / 2.0 + std::abs(right_overhang - left_overhang) / 2.0;
  const double spacing = vehicle_length / static_cast<double>(count - 1);
  std::vector<PathOptimizerCircle> circles;
  circles.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    circles.push_back({spacing * static_cast<double>(i) - rear_overhang, radius});
  }
  return circles;
}

PathOptimizerQpProblem buildPathOptimizerQp(
    const std::vector<PathOptimizerReference> &references,
    const std::vector<PathOptimizerCircle> &circles,
    const bool goal_contained, const PathOptimizerQpParameters &p) {
  validate(references, circles, p);
  const Eigen::Index n = static_cast<Eigen::Index>(references.size());
  const Eigen::Index circle_count = static_cast<Eigen::Index>(circles.size());
  const Eigen::Index nx = 2 * n;
  const Eigen::Index nu = n - 1;
  const Eigen::Index ns = p.soft_constraints ? n * (p.shared_slack ? 1 : circle_count) : 0;
  const Eigen::Index nv = nx + nu + ns;
  Eigen::Index fixed_count = 0;
  for (const auto &reference : references) fixed_count += reference.fixed_state.has_value();
  const Eigen::Index rows = nx +
      (p.soft_constraints ? 3 * n * circle_count : 0) +
      (p.hard_constraints ? n * circle_count : 0) +
      2 * fixed_count + (p.steering_limit_constraint ? nu : 0);

  PathOptimizerQpProblem result;
  result.state_variables = nx;
  result.steering_variables = nu;
  result.slack_variables = ns;
  result.hessian = Eigen::MatrixXd::Zero(nv, nv);
  result.gradient = Eigen::VectorXd::Zero(nv);
  result.constraints = Eigen::MatrixXd::Zero(rows, nv);
  result.lower_bound = Eigen::VectorXd::Constant(rows, -kInfinity);
  result.upper_bound = Eigen::VectorXd::Constant(rows, kInfinity);

  Eigen::MatrixXd q = Eigen::MatrixXd::Zero(nx, nx);
  Eigen::MatrixXd transform = Eigen::MatrixXd::Zero(nx, nx);
  Eigen::VectorXd transform_offset = Eigen::VectorXd::Zero(nx);
  for (Eigen::Index i = 0; i < n; ++i) {
    const auto &reference = references[static_cast<std::size_t>(i)];
    const bool terminal = i == n - 1;
    q(2 * i, 2 * i) = terminal ?
        (goal_contained ? p.goal_lateral_weight : p.terminal_lateral_weight) :
        blend(p.lateral_weight, p.avoidance_lateral_weight,
              reference.avoidance_cost);
    q(2 * i + 1, 2 * i + 1) = terminal ?
        (goal_contained ? p.goal_yaw_weight : p.terminal_yaw_weight) :
        blend(p.yaw_weight, p.avoidance_yaw_weight,
              reference.avoidance_cost);
    const double c = std::cos(reference.alpha);
    transform(2 * i, 2 * i) = c;
    transform(2 * i, 2 * i + 1) = p.optimization_center_offset * c;
    transform(2 * i + 1, 2 * i + 1) = 1.0;
    transform_offset(2 * i) =
        -p.optimization_center_offset * std::sin(reference.alpha);
  }
  result.hessian.topLeftCorner(nx, nx).noalias() =
      transform.transpose() * q * transform;
  result.gradient.head(nx).noalias() =
      transform.transpose() * q * transform_offset;
  for (Eigen::Index i = 0; i < nu; ++i) {
    const double avoidance = references[static_cast<std::size_t>(i)].avoidance_cost;
    result.hessian(nx + i, nx + i) +=
        blend(p.steering_weight, p.avoidance_steering_weight, avoidance);
    if (i + 1 < nu) {
      result.hessian(nx + i, nx + i) += p.steering_rate_weight;
      result.hessian(nx + i + 1, nx + i + 1) += p.steering_rate_weight;
      result.hessian(nx + i, nx + i + 1) -= p.steering_rate_weight;
      result.hessian(nx + i + 1, nx + i) -= p.steering_rate_weight;
    }
  }
  if (ns) result.gradient.tail(ns).setConstant(p.soft_collision_weight);

  std::vector<double> arc_lengths;
  arc_lengths.reserve(static_cast<std::size_t>(nu));
  for (Eigen::Index i = 0; i < nu; ++i) {
    arc_lengths.push_back(references[static_cast<std::size_t>(i)].arc_length_to_next);
  }
  const auto equation = buildPathOptimizerStateEquation(
      arc_lengths, p.wheelbase, p.maximum_steering);
  result.constraints.topLeftCorner(nx, nx).setIdentity();
  result.constraints.topLeftCorner(nx, nx) -= equation.transition;
  result.constraints.block(0, nx, nx, nu) = -equation.steering;
  result.lower_bound.head(nx) = equation.offset;
  result.upper_bound.head(nx) = equation.offset;

  Eigen::Index row = nx;
  for (Eigen::Index circle_index = 0; circle_index < circle_count; ++circle_index) {
    const auto &circle = circles[static_cast<std::size_t>(circle_index)];
    const double bound_offset = p.vehicle_width / 2.0 - circle.radius;
    for (Eigen::Index i = 0; i < n; ++i) {
      const auto &reference = references[static_cast<std::size_t>(i)];
      const double beta = reference.beta[static_cast<std::size_t>(circle_index)];
      const double lateral = std::cos(beta);
      const double yaw = circle.longitudinal_offset * lateral;
      const double shift = circle.longitudinal_offset * std::sin(beta);
      const auto &bound = reference.circle_bounds[static_cast<std::size_t>(circle_index)];
      const double lower = bound.lower - bound_offset;
      const double upper = bound.upper + bound_offset;
      if (p.soft_constraints) {
        const Eigen::Index slack = nx + nu +
            (p.shared_slack ? i : circle_index * n + i);
        result.constraints(row + i, 2 * i) = lateral;
        result.constraints(row + i, 2 * i + 1) = yaw;
        result.constraints(row + i, slack) = 1.0;
        result.lower_bound(row + i) = lower - shift;
        result.constraints(row + n + i, 2 * i) = -lateral;
        result.constraints(row + n + i, 2 * i + 1) = -yaw;
        result.constraints(row + n + i, slack) = 1.0;
        result.lower_bound(row + n + i) = shift - upper;
        result.constraints(row + 2 * n + i, slack) = 1.0;
        result.lower_bound(row + 2 * n + i) = 0.0;
      }
      if (p.hard_constraints) {
        const Eigen::Index hard_row = row + (p.soft_constraints ? 3 * n : 0) + i;
        result.constraints(hard_row, 2 * i) = lateral;
        result.constraints(hard_row, 2 * i + 1) = yaw;
        result.lower_bound(hard_row) = lower - shift;
        result.upper_bound(hard_row) = upper - shift;
      }
    }
    row += (p.soft_constraints ? 3 * n : 0) + (p.hard_constraints ? n : 0);
  }
  for (Eigen::Index i = 0; i < n; ++i) {
    const auto &fixed = references[static_cast<std::size_t>(i)].fixed_state;
    if (!fixed) continue;
    result.constraints.block<2, 2>(row, 2 * i).setIdentity();
    result.lower_bound.segment<2>(row) = *fixed;
    result.upper_bound.segment<2>(row) = *fixed;
    row += 2;
  }
  if (p.steering_limit_constraint) {
    result.constraints.block(row, nx, nu, nu).setIdentity();
    for (Eigen::Index i = 0; i < nu; ++i) {
      const double reference_steering = std::atan2(
          p.wheelbase * references[static_cast<std::size_t>(i)].curvature, 1.0);
      result.lower_bound(row + i) = reference_steering - p.maximum_steering;
      result.upper_bound(row + i) = reference_steering + p.maximum_steering;
    }
  }
  return result;
}

}  // planning 네임스페이스
