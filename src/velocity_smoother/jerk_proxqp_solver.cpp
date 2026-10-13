// Copyright 2021 Tier IV, Inc.
// SPDX-License-Identifier: Apache-2.0
// 원본 스무더와 같은 ProxQP 희소 행렬 풀이 설정입니다.

#include "planning/jerk_qp_solver.hpp"

#include <proxsuite/proxqp/sparse/sparse.hpp>

#include <Eigen/Sparse>

#include <cmath>
#include <stdexcept>

namespace planning {

JerkQpSolveResult solveJerkQpProx(
    const JerkQpProblem &p, const JerkQpSolveOptions &options) {
  const auto n = p.hessian.rows();
  const auto m = p.constraints.rows();
  if (n == 0 || m == 0 || p.hessian.cols() != n ||
      p.constraints.cols() != n || p.linear_cost.size() != n ||
      p.lower_bound.size() != m || p.upper_bound.size() != m ||
      !p.hessian.allFinite() || !p.constraints.allFinite() ||
      !p.linear_cost.allFinite() || !p.lower_bound.allFinite() ||
      !p.upper_bound.allFinite() ||
      (p.lower_bound.array() > p.upper_bound.array()).any() ||
      options.max_iterations <= 0 ||
      !std::isfinite(options.absolute_tolerance) ||
      !(options.absolute_tolerance > 0.0) ||
      !std::isfinite(options.relative_tolerance) ||
      !(options.relative_tolerance > 0.0)) {
    throw std::invalid_argument("잘못된 ProxQP 문제 또는 풀이 설정");
  }

  proxsuite::proxqp::sparse::QP<double, int> solver(n, 0, m);
  solver.settings.max_iter = options.max_iterations;
  solver.settings.eps_abs = options.absolute_tolerance;
  solver.settings.eps_rel = options.relative_tolerance;
  solver.settings.verbose = false;
  solver.settings.initial_guess =
      proxsuite::proxqp::InitialGuessStatus::NO_INITIAL_GUESS;

  const Eigen::SparseMatrix<double> hessian = p.hessian.sparseView();
  const Eigen::SparseMatrix<double> constraints = p.constraints.sparseView();
  solver.init(hessian, p.linear_cost, proxsuite::nullopt,
              proxsuite::nullopt, constraints, p.lower_bound, p.upper_bound);
  solver.solve();

  JerkQpSolveResult result;
  result.backend = JerkQpBackend::ProxQP;
  result.iterations = solver.results.info.iter;
  result.primal_residual = solver.results.info.pri_res;
  result.dual_residual = solver.results.info.dua_res;
  result.solution = solver.results.x;
  result.solved = solver.results.info.status ==
                      proxsuite::proxqp::QPSolverOutput::PROXQP_SOLVED &&
                  result.solution.allFinite();
  return result;
}

}  // planning 네임스페이스
