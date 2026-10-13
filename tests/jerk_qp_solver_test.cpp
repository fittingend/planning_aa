#include "planning/jerk_qp_solver.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>

namespace {
void check(bool condition, const char *message) {
  if (!condition) {
    std::cerr << "실패: " << message << '\n';
    std::exit(EXIT_FAILURE);
  }
}
}  // 익명 네임스페이스

int main() {
  const planning::Trajectory points{{0.0, 0.0, 2.0F, 0.0F},
                                    {1.0, 0.0, 2.0F, 0.0F},
                                    {2.0, 0.0, 0.0F, 0.0F}};
  const auto qp = planning::buildJerkQpProblem(points, 1.0, 0.0, {});
  const auto result = planning::solveJerkQp(qp);
  check(result.solved, "QP 풀이가 수렴해야 합니다");
#if defined(PLANNING_HAS_PROXQP)
  check(result.backend == planning::JerkQpBackend::ProxQP,
        "제공된 ProxQP 구현을 사용해야 합니다");
  planning::JerkQpSolveOptions admm_options;
  admm_options.absolute_tolerance = 1.0e-6;
  const auto admm = planning::solveJerkQpAdmm(qp, admm_options);
  check(admm.solved, "ADMM 비교 풀이가 수렴해야 합니다");
  const auto objective = [&](const Eigen::VectorXd &x) {
    return 0.5 * x.dot(qp.hessian * x) + qp.linear_cost.dot(x);
  };
  check(std::fabs(objective(result.solution) - objective(admm.solution)) < 1.0e-3,
        "ProxQP와 ADMM의 목적함수 값이 가까워야 합니다");
#endif
  check(result.solution.size() == qp.hessian.rows(), "해의 크기가 맞아야 합니다");
  check(result.primal_residual < 1.0e-4 && result.dual_residual < 1.0e-3,
        "제약과 정지 조건 잔차가 작아야 합니다");
  const auto values = qp.constraints * result.solution;
  for (Eigen::Index i = 0; i < values.size(); ++i) {
    check(values[i] >= qp.lower_bound[i] - 1.0e-4 &&
              values[i] <= qp.upper_bound[i] + 1.0e-4,
          "모든 제약을 만족해야 합니다");
  }
  check(std::fabs(result.solution[0] - 1.0) < 1.0e-4 &&
            std::fabs(result.solution[3]) < 1.0e-4,
        "초기 속도 제곱과 가속도를 유지해야 합니다");
  std::cout << "저크 QP 풀이 통과\n";
}
