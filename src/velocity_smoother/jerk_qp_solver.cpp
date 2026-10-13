// Copyright 2021 Tier IV, Inc.
// SPDX-License-Identifier: Apache-2.0
// 상자 제약 QP를 푸는 Eigen 기반 ADMM 구현입니다.

#include "planning/jerk_qp_solver.hpp"

#include <Eigen/Sparse>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace planning {
namespace {

void validate(const JerkQpProblem &p, const JerkQpSolveOptions &o) {
  const auto n = p.hessian.rows();
  const auto m = p.constraints.rows();
  if (n == 0 || m == 0 || p.hessian.cols() != n ||
      p.constraints.cols() != n || p.linear_cost.size() != n ||
      p.lower_bound.size() != m || p.upper_bound.size() != m ||
      !p.hessian.allFinite() || !p.constraints.allFinite() ||
      !p.linear_cost.allFinite() || !p.lower_bound.allFinite() ||
      !p.upper_bound.allFinite() ||
      (p.lower_bound.array() > p.upper_bound.array()).any() ||
      o.max_iterations <= 0 || !std::isfinite(o.penalty) || o.penalty <= 0.0 ||
      !std::isfinite(o.absolute_tolerance) || o.absolute_tolerance <= 0.0 ||
      !std::isfinite(o.relative_tolerance) || o.relative_tolerance <= 0.0 ||
      !std::isfinite(o.admm_absolute_residual_limit) ||
      o.admm_absolute_residual_limit <= 0.0) {
    throw std::invalid_argument("잘못된 QP 문제 또는 풀이 설정");
  }
}

}  // 익명 네임스페이스

JerkQpSolveResult solveJerkQpAdmm(
    const JerkQpProblem &p, const JerkQpSolveOptions &o) {
  validate(p, o);
  const auto n = p.hessian.rows();
  const auto m = p.constraints.rows();
  const Eigen::SparseMatrix<double> a = p.constraints.sparseView();
  const Eigen::SparseMatrix<double> at = a.transpose();
  constexpr double sigma = 1.0e-6;
  Eigen::SparseMatrix<double> k = p.hessian.sparseView();
  k += o.penalty * (at * a);
  for (Eigen::Index i = 0; i < n; ++i) k.coeffRef(i, i) += sigma;
  k.makeCompressed();
  Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> factor;
  factor.compute(k);
  if (factor.info() != Eigen::Success) {
    throw std::runtime_error("QP 행렬 분해 실패");
  }

  Eigen::VectorXd x = Eigen::VectorXd::Zero(n);
  Eigen::VectorXd z = Eigen::VectorXd::Zero(m);
  Eigen::VectorXd y = Eigen::VectorXd::Zero(m);
  JerkQpSolveResult result;
  for (int iteration = 1; iteration <= o.max_iterations; ++iteration) {
    const Eigen::VectorXd rhs = -p.linear_cost + sigma * x +
                                at * (o.penalty * z - y);
    x = factor.solve(rhs);
    if (factor.info() != Eigen::Success || !x.allFinite()) break;
    const Eigen::VectorXd ax = a * x;
    const Eigen::VectorXd previous_z = z;
    z = (ax + y / o.penalty).cwiseMax(p.lower_bound).cwiseMin(p.upper_bound);
    y += o.penalty * (ax - z);
    const Eigen::VectorXd stationarity = p.hessian * x + p.linear_cost + at * y;
    result.iterations = iteration;
    result.primal_residual = (ax - z).lpNorm<Eigen::Infinity>();
    result.dual_residual = stationarity.lpNorm<Eigen::Infinity>();
    const double primal_limit = o.absolute_tolerance + o.relative_tolerance *
        std::max(ax.lpNorm<Eigen::Infinity>(), z.lpNorm<Eigen::Infinity>());
    const double dual_limit = o.absolute_tolerance + o.relative_tolerance *
        std::max({(p.hessian * x).lpNorm<Eigen::Infinity>(),
                  (at * y).lpNorm<Eigen::Infinity>(),
                  p.linear_cost.lpNorm<Eigen::Infinity>()});
    if (result.primal_residual <= std::min(
            primal_limit, o.admm_absolute_residual_limit) &&
        result.dual_residual <= std::min(
            dual_limit, o.admm_absolute_residual_limit)) {
      result.solved = true;
      break;
    }
    if (!z.allFinite() || !y.allFinite() ||
        !(previous_z - z).allFinite()) break;
  }
  result.solution = x;
  return result;
}

#if defined(PLANNING_HAS_PROXQP)
JerkQpSolveResult solveJerkQpProx(
    const JerkQpProblem &problem, const JerkQpSolveOptions &options);
#endif

JerkQpSolveResult solveJerkQp(
    const JerkQpProblem &problem, const JerkQpSolveOptions &options) {
#if defined(PLANNING_HAS_PROXQP)
  return solveJerkQpProx(problem, options);
#else
  return solveJerkQpAdmm(problem, options);
#endif
}

}  // planning 네임스페이스
