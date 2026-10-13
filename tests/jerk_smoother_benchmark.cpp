// 독립 스무더의 합성 정지 경로 계산 시간을 같은 프로세스에서 반복 측정합니다.

#include "planning/jerk_smoother.hpp"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

int main(int argc, char **argv) {
  const int iterations = argc > 1 ? std::atoi(argv[1]) : 50;
  if (iterations < 1) return 2;
  planning::Trajectory path;
  if (argc > 2) {
    std::ifstream input(argv[2]);
    if (!input) return 2;
    std::string line;
    while (std::getline(input, line)) {
      std::istringstream stream(line);
      double x = 0.0;
      double y = 0.0;
      float velocity = 0.0F;
      float acceleration = 0.0F;
      char comma = 0;
      if (!(stream >> x >> comma >> y >> comma >> velocity >> comma >>
            acceleration)) return 2;
      path.push_back({x, y, velocity, acceleration});
    }
  } else {
    for (int index = 0; index <= 100; ++index) {
      path.push_back({static_cast<double>(index), 0.0,
                      index == 100 ? 0.0F : 10.0F, 0.0F});
    }
  }
  if (path.size() < 2) return 2;
  const double initial_velocity = argc > 3 ? std::atof(argv[3]) : 5.0;
  planning::JerkSmootherParameters parameters;
  if (argc > 4) parameters.solver.penalty = std::atof(argv[4]);
  if (argc > 6) parameters.solver.relative_tolerance = std::atof(argv[6]);
  const auto warmup = planning::runJerkSmoother(
      path, initial_velocity, 0.0, parameters);
  if (!warmup.output || !warmup.qp_result.solved) {
    std::cerr << "예열 풀이 실패: 반복 " << warmup.qp_result.iterations
              << "회, 원시 잔차 " << warmup.qp_result.primal_residual
              << ", 쌍대 잔차 " << warmup.qp_result.dual_residual << '\n';
    return 1;
  }
  if (argc > 5) {
    const auto problem = planning::buildJerkQpProblem(
        warmup.optimization_input, initial_velocity, 0.0, parameters.qp);
    const auto &solution = warmup.qp_result.solution;
    const auto constrained = problem.constraints * solution;
    const auto violation = (
        constrained.cwiseMax(problem.lower_bound)
                   .cwiseMin(problem.upper_bound) - constrained)
                               .lpNorm<Eigen::Infinity>();
    const double objective = 0.5 * solution.dot(problem.hessian * solution) +
                             problem.linear_cost.dot(solution);
    std::cout << "풀이 반복 " << warmup.qp_result.iterations
              << "회, 원시 잔차 " << warmup.qp_result.primal_residual
              << ", 쌍대 잔차 " << warmup.qp_result.dual_residual
              << ", 제약 위반 " << violation
              << ", 목적함수 " << objective << '\n';
    std::ofstream output(argv[5]);
    if (!output) return 2;
    output.precision(17);
    for (const auto &point : *warmup.output) {
      output << point.x << ',' << point.y << ','
             << point.longitudinal_velocity_mps << ','
             << point.acceleration_mps2 << '\n';
    }
  }
  std::vector<double> elapsed_ms;
  elapsed_ms.reserve(static_cast<std::size_t>(iterations));
  for (int iteration = 0; iteration < iterations; ++iteration) {
    const auto start = std::chrono::steady_clock::now();
    const auto result = planning::runJerkSmoother(
        path, initial_velocity, 0.0, parameters);
    const auto end = std::chrono::steady_clock::now();
    if (!result.output || !result.qp_result.solved) return 1;
    elapsed_ms.push_back(
        std::chrono::duration<double, std::milli>(end - start).count());
  }
  std::sort(elapsed_ms.begin(), elapsed_ms.end());
  const auto percentile = [&](double fraction) {
    const auto index = static_cast<std::size_t>(
        fraction * static_cast<double>(elapsed_ms.size() - 1));
    return elapsed_ms[index];
  };
  std::cout << "입력 " << path.size() << "점, 반복 " << iterations
            << "회, 중앙값 " << percentile(0.5)
            << "ms, 95백분위 " << percentile(0.95)
            << "ms, 최대 " << elapsed_ms.back() << "ms\n";
}
