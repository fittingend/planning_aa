#include "planning/jerk_qp_problem.hpp"

#include <cstdlib>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {
void check(bool condition, const std::string &message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    std::exit(EXIT_FAILURE);
  }
}
}  // 익명 네임스페이스

int main() {
  const planning::Trajectory resampled{{0.0, 0.0, 2.0F, 0.0F},
                                       {1.0, 0.0, 2.0F, 0.0F},
                                       {2.0, 0.0, 2.0F, 0.0F}};
  const auto qp = planning::buildJerkQpProblem(resampled, 1.0, 0.5, {});
  check(qp.optimized_points == 3 && qp.hessian.rows() == 15 &&
            qp.constraints.rows() == 13 && qp.constraints.cols() == 15,
        "QP dimensions retain source variable and constraint layout");
  check(qp.upper_bound[2] == 0.0 && qp.constraints(2, 2) == 1.0,
        "terminal zero speed appears as squared-speed constraint");
  check(qp.constraints(3, 3) == 1.0 && qp.constraints(3, 9) == -1.0 &&
            qp.lower_bound[3] == -1.0 && qp.upper_bound[3] == 1.0,
        "moving-point acceleration has source soft bounds");
  check(qp.lower_bound[5] == 0.0 && qp.upper_bound[5] == 0.0,
        "stop-point acceleration is fixed to configured stop deceleration");
  check(qp.constraints(10, 0) == 1.0 && qp.upper_bound[10] == 1.0 &&
            qp.constraints(11, 3) == 1.0 && qp.lower_bound[11] == 0.5,
        "initial squared speed and acceleration are fixed");
  check(qp.hessian(3, 3) == 40.0 && qp.hessian(6, 6) == 100000.0,
        "jerk and slack costs use configured weights");
  check(qp.constraints.row(12).isZero() && qp.lower_bound[12] == 0.0 &&
            qp.upper_bound[12] == 0.0,
        "source's extra zero equality row is retained");

  bool rejected = false;
  try {
    planning::buildJerkQpProblem({resampled[0]}, 1.0, 0.0, {});
  } catch (const std::invalid_argument &) {
    rejected = true;
  }
  check(rejected, "single-point trajectory cannot enter the QP stage");

  const planning::Trajectory with_tail{{0.0, 0.0, 2.0F, 0.0F},
                                       {1.0, 0.0, 0.0F, 0.0F},
                                       {2.0, 0.0, 2.0F, 0.0F}};
  const auto tail_qp = planning::buildJerkQpProblem(with_tail, 1.0, 0.5, {});
  Eigen::VectorXd solution = Eigen::VectorXd::Zero(5 * tail_qp.optimized_points);
  solution[0] = 1.0;
  solution[1] = -0.01;
  solution[2] = 0.5;
  solution[3] = -0.5;
  const auto converted = planning::convertJerkQpSolution(with_tail, tail_qp, solution, -0.2);
  check(converted.size() == 3 && converted[0].longitudinal_velocity_mps == 1.0F &&
            converted[1].longitudinal_velocity_mps == 0.0F &&
            converted[0].acceleration_mps2 == 0.5F &&
            converted[1].acceleration_mps2 == -0.5F,
        "QP squared speeds and accelerations are converted on optimized points");
  check(converted[2].longitudinal_velocity_mps == 0.0F &&
            std::fabs(converted[2].acceleration_mps2 + 0.2F) < 1.0e-6F,
        "points after the first stop receive source stop state");
  rejected = false;
  solution[0] = std::nan("");
  try {
    planning::convertJerkQpSolution(with_tail, tail_qp, solution, -0.2);
  } catch (const std::invalid_argument &) {
    rejected = true;
  }
  check(rejected, "nonfinite QP solution is rejected");

  std::cout << "jerk QP construction passed\n";
}
