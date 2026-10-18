// Copyright 2021 Tier IV, Inc.
// SPDX-License-Identifier: Apache-2.0
// JerkFilteredSmoother::apply의 계산 순서를 이관했습니다.

#include "planning/jerk_smoother.hpp"

#include <chrono>
#include <cmath>
#include <stdexcept>

namespace planning {

JerkSmootherResult runJerkSmoother(
    const Trajectory &input, double initial_velocity, double initial_acceleration,
    const JerkSmootherParameters &p) {
  if (input.empty() || !std::isfinite(initial_velocity) ||
      !std::isfinite(initial_acceleration)) {
    throw std::invalid_argument("스무더에는 유효한 궤적과 초기 상태가 필요합니다");
  }
  JerkSmootherResult result;
  if (input.size() == 1) {
    auto output = input;
    output.front().longitudinal_velocity_mps = static_cast<float>(initial_velocity);
    output.front().acceleration_mps2 = static_cast<float>(initial_acceleration);
    result.stages.forward = output;
    result.stages.backward = output;
    result.stages.merged = output;
    result.output = output;
    return result;
  }

  result.stages = runJerkFilterStage(
      input, initial_velocity, initial_acceleration, p.filter);
  result.optimization_input = resampleJerkTrajectory(
      result.stages.merged, initial_velocity, p.resampling);
  result.optimization_input.back().longitudinal_velocity_mps = 0.0F;
  if (result.optimization_input.size() == 1) {
    auto output = input;
    output.front().longitudinal_velocity_mps = static_cast<float>(initial_velocity);
    output.front().acceleration_mps2 = static_cast<float>(initial_acceleration);
    result.output = output;
    return result;
  }

  // JerkFilteredSmoother::apply initializes output to the resampled path
  // before solving. VelocitySmootherNode keeps that path when QP fails.
  result.output = result.optimization_input;

  const auto problem = buildJerkQpProblem(
      result.optimization_input, initial_velocity, initial_acceleration, p.qp);
  const auto first_solve_started = std::chrono::steady_clock::now();
  result.qp_result = solveJerkQp(problem, p.solver);
  result.qp_first_solve_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - first_solve_started).count();
  result.qp_first_iterations = result.qp_result.iterations;
  if (!result.qp_result.solved &&
      result.qp_result.backend == JerkQpBackend::Admm &&
      p.solver.penalty == 1.0) {
    auto retry = p.solver;
    retry.penalty = 0.1;
    result.qp_retried = true;
    const auto retry_started = std::chrono::steady_clock::now();
    result.qp_result = solveJerkQpAdmm(problem, retry);
    result.qp_retry_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - retry_started).count();
  }
  if (result.qp_result.solved) {
    result.output = convertJerkQpSolution(
        result.optimization_input, problem, result.qp_result.solution,
        p.qp.stop_deceleration);
  }
  return result;
}

}  // planning 네임스페이스
