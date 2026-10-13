// Copyright 2021 Tier IV, Inc.
// SPDX-License-Identifier: Apache-2.0
// autoware_velocity_smoother/src/smoother/jerk_filtered_smoother.cpp를 바탕으로 작성했습니다.

#include "planning/jerk_qp_problem.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace planning {

JerkQpProblem buildJerkQpProblem(
    const Trajectory &resampled, double initial_velocity, double initial_acceleration,
    const JerkQpParameters &p) {
  if (resampled.size() < 2) {
    throw std::invalid_argument("QP requires at least two resampled points");
  }

  auto trajectory = resampled;
  trajectory.back().longitudinal_velocity_mps = 0.0F;
  std::size_t stop = 1;
  while (stop < trajectory.size() && trajectory[stop].longitudinal_velocity_mps != 0.0F) {
    ++stop;
  }
  if (stop == trajectory.size()) {
    throw std::invalid_argument("QP requires a stop point");
  }
  const std::size_t n = stop + 1;
  const Eigen::Index count = static_cast<Eigen::Index>(n);
  const Eigen::Index b0 = 0;
  const Eigen::Index a0 = count;
  const Eigen::Index delta0 = 2 * count;
  const Eigen::Index sigma0 = 3 * count;
  const Eigen::Index gamma0 = 4 * count;

  JerkQpProblem problem;
  problem.optimized_points = n;
  problem.hessian = Eigen::MatrixXd::Zero(5 * count, 5 * count);
  // 원본의 추가 등식 제약 행 1개를 유지합니다(총 4*N+1행).
  problem.constraints = Eigen::MatrixXd::Zero(4 * count + 1, 5 * count);
  problem.linear_cost = Eigen::VectorXd::Zero(5 * count);
  problem.lower_bound = Eigen::VectorXd::Zero(4 * count + 1);
  problem.upper_bound = Eigen::VectorXd::Zero(4 * count + 1);

  std::vector<double> ds(trajectory.size() - 1);
  for (std::size_t i = 0; i < ds.size(); ++i) {
    ds[i] = std::hypot(trajectory[i + 1].x - trajectory[i].x,
                       trajectory[i + 1].y - trajectory[i].y);
    if (!(ds[i] > 0.0)) {
      throw std::invalid_argument("QP requires positive point spacing");
    }
  }

  for (std::size_t i = 0; i < n - 1; ++i) {
    const double reference_velocity =
        0.5 * (trajectory[i].longitudinal_velocity_mps +
               trajectory[i + 1].longitudinal_velocity_mps);
    const double interval = std::max(ds[i], 0.0001);
    const double scaled = reference_velocity / interval;
    const double weight = p.jerk_weight * scaled * scaled * interval;
    const auto j = static_cast<Eigen::Index>(i);
    problem.hessian(a0 + j, a0 + j) += weight;
    problem.hessian(a0 + j, a0 + j + 1) -= weight;
    problem.hessian(a0 + j + 1, a0 + j) -= weight;
    problem.hessian(a0 + j + 1, a0 + j + 1) += weight;
  }

  for (std::size_t i = 0; i < n; ++i) {
    const auto j = static_cast<Eigen::Index>(i);
    const double maximum = trajectory[i].longitudinal_velocity_mps;
    if (maximum > 0.01) {
      double velocity_term = -1.0 / (maximum * maximum);
      if (i < n - 1) {
        velocity_term *= std::max(ds[i], 0.0001);
      }
      problem.linear_cost(b0 + j) += velocity_term;
    }
    problem.hessian(delta0 + j, delta0 + j) += p.over_velocity_weight;
    problem.hessian(sigma0 + j, sigma0 + j) += p.over_acceleration_weight;
    problem.hessian(gamma0 + j, gamma0 + j) += p.over_jerk_weight;
  }

  Eigen::Index row = 0;
  for (std::size_t i = 0; i < n; ++i, ++row) {
    const auto j = static_cast<Eigen::Index>(i);
    problem.constraints(row, b0 + j) = 1.0;
    problem.constraints(row, delta0 + j) = -1.0;
    const double maximum = trajectory[i].longitudinal_velocity_mps;
    problem.upper_bound[row] = maximum * maximum;
  }
  for (std::size_t i = 0; i < n; ++i, ++row) {
    const auto j = static_cast<Eigen::Index>(i);
    problem.constraints(row, a0 + j) = 1.0;
    problem.constraints(row, sigma0 + j) = -1.0;
    if (trajectory[i].longitudinal_velocity_mps < 1.0e-3) {
      problem.upper_bound[row] = p.stop_deceleration;
      problem.lower_bound[row] = p.stop_deceleration;
    } else {
      problem.upper_bound[row] = p.max_acceleration;
      problem.lower_bound[row] = p.min_deceleration;
    }
  }
  for (std::size_t i = 0; i < n - 1; ++i, ++row) {
    const auto j = static_cast<Eigen::Index>(i);
    const double reference_velocity =
        0.5 * (trajectory[i].longitudinal_velocity_mps +
               trajectory[i + 1].longitudinal_velocity_mps);
    problem.constraints(row, a0 + j) = -reference_velocity;
    problem.constraints(row, a0 + j + 1) = reference_velocity;
    problem.constraints(row, gamma0 + j) = -ds[i];
    problem.upper_bound[row] = p.max_jerk * ds[i];
    problem.lower_bound[row] = p.min_jerk * ds[i];
  }
  for (std::size_t i = 0; i < n - 1; ++i, ++row) {
    const auto j = static_cast<Eigen::Index>(i);
    problem.constraints(row, b0 + j) = -1.0;
    problem.constraints(row, b0 + j + 1) = 1.0;
    problem.constraints(row, a0 + j) = -2.0 * ds[i];
  }
  problem.constraints(row, b0) = 1.0;
  problem.upper_bound[row] = initial_velocity * initial_velocity;
  problem.lower_bound[row] = initial_velocity * initial_velocity;
  ++row;
  problem.constraints(row, a0) = 1.0;
  problem.upper_bound[row] = initial_acceleration;
  problem.lower_bound[row] = initial_acceleration;

  return problem;
}

Trajectory convertJerkQpSolution(
    const Trajectory &resampled, const JerkQpProblem &problem,
    const Eigen::VectorXd &solution, double stop_deceleration) {
  const std::size_t n = problem.optimized_points;
  if (n < 2 || n > resampled.size() ||
      solution.size() != static_cast<Eigen::Index>(5 * n) ||
      !solution.allFinite() || !std::isfinite(stop_deceleration)) {
    throw std::invalid_argument("invalid jerk QP result");
  }
  auto output = resampled;
  for (std::size_t i = 0; i < n; ++i) {
    const auto j = static_cast<Eigen::Index>(i);
    output[i].longitudinal_velocity_mps =
        static_cast<float>(std::sqrt(std::max(solution[j], 0.0)));
    output[i].acceleration_mps2 =
        static_cast<float>(solution[static_cast<Eigen::Index>(n) + j]);
  }
  for (std::size_t i = n; i < output.size(); ++i) {
    output[i].longitudinal_velocity_mps = 0.0F;
    output[i].acceleration_mps2 = static_cast<float>(stop_deceleration);
  }
  return output;
}

}  // planning 네임스페이스
