// Copyright 2021 Tier IV, Inc.
// SPDX-License-Identifier: Apache-2.0
// Eigen 기반 QP 풀이 경로입니다.

#pragma once

#include "planning/jerk_qp_problem.hpp"

#include <Eigen/Core>

namespace planning {

struct JerkQpSolveOptions {
  int max_iterations = 20000;
  double absolute_tolerance = 1.0e-8;
  double relative_tolerance = 1.0e-6;
  // ADMM 보조 풀이에만 사용합니다.
  double penalty = 1.0;
  // 큰 문제에서 상대 오차만 통과한 부정확한 해를 거부합니다.
  double admm_absolute_residual_limit = 1.0e-3;
};

enum class JerkQpBackend { ProxQP, Admm };

struct JerkQpSolveResult {
  bool solved = false;
  JerkQpBackend backend = JerkQpBackend::Admm;
  int iterations = 0;
  double primal_residual = 0.0;
  double dual_residual = 0.0;
  Eigen::VectorXd solution;
};

// 제공된 ProxSuite가 있으면 원본과 같은 ProxQP를 사용합니다.
// 없으면 Eigen 기반 ADMM 풀이를 사용합니다.
// 수렴하지 않으면 solved가 거짓이므로 결과를 발행하지 않아야 합니다.
JerkQpSolveResult solveJerkQp(
    const JerkQpProblem &problem, const JerkQpSolveOptions &options = {});

// 비교와 비상 대체를 위한 Eigen 기반 ADMM 풀이입니다.
JerkQpSolveResult solveJerkQpAdmm(
    const JerkQpProblem &problem, const JerkQpSolveOptions &options = {});

}  // planning 네임스페이스
