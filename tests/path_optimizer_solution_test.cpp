// SPDX-License-Identifier: Apache-2.0
// 최적화 상태를 위치·방향·속도로 변환하고 유효성 경계를 검사합니다.

#include "planning/path_optimizer_solution.hpp"

#include <cmath>
#include <stdexcept>

namespace {
void check(const bool condition) {
  if (!condition) throw std::runtime_error("경로 최적화 궤적 변환 검사 실패");
}
}  // 네임스페이스

int main() {
  planning::Trajectory references(2);
  references[0].x = 1.0;
  references[0].y = 2.0;
  references[0].z = 3.0;
  references[0].longitudinal_velocity_mps = 4.0F;
  references[0].orientation_z = std::sin(0.25);
  references[0].orientation_w = std::cos(0.25);
  references[1].x = 5.0;
  references[1].longitudinal_velocity_mps = 6.0F;
  planning::PathOptimizerQpProblem problem;
  problem.state_variables = 4;
  problem.steering_variables = 1;
  problem.hessian = Eigen::MatrixXd::Zero(5, 5);
  Eigen::VectorXd solution(5);
  solution << 0.2, 0.1, -0.1, -0.2, 0.0;
  const auto trajectory = planning::pathOptimizerSolutionToTrajectory(
      references, problem, solution);
  check(trajectory.has_value());
  check(std::abs((*trajectory)[0].x - (1.0 - 0.2 * std::sin(0.5))) < 1e-12);
  check(std::abs((*trajectory)[0].y - (2.0 + 0.2 * std::cos(0.5))) < 1e-12);
  check((*trajectory)[0].z == 3.0);
  check((*trajectory)[0].longitudinal_velocity_mps == 4.0F);
  check(std::abs((*trajectory)[0].orientation_z - std::sin(0.3)) < 1e-12);
  check(std::abs((*trajectory)[1].y + 0.1) < 1e-12);
  check((*trajectory)[1].longitudinal_velocity_mps == 6.0F);

  planning::PathOptimizerValidation validation;
  validation.enabled = true;
  validation.maximum_lateral_error = 0.15;
  check(!planning::pathOptimizerSolutionToTrajectory(
      references, problem, solution, validation));
  validation.maximum_lateral_error = 1.0;
  validation.maximum_yaw_error = 0.15;
  check(!planning::pathOptimizerSolutionToTrajectory(
      references, problem, solution, validation));
}
