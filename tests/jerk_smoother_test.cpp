#include "planning/jerk_smoother.hpp"

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
  const planning::Trajectory input{{0.0, 0.0, 2.0F, 0.0F},
                                   {1.0, 0.0, 2.0F, 0.0F},
                                   {2.0, 0.0, 0.0F, 0.0F}};
  planning::JerkSmootherParameters p;
  p.resampling.min_trajectory_length = 4.0;
  const auto result = planning::runJerkSmoother(input, 1.0, 0.0, p);
  check(result.output.has_value(), "전체 스무더 계산이 끝나야 합니다");
  check(result.qp_result.solved, "QP 풀이가 수렴해야 합니다");
  check(result.stages.forward.size() == input.size() &&
            result.stages.backward.size() == input.size() &&
            result.stages.merged.size() == input.size(),
        "세 필터 단계를 보관해야 합니다");
  check(std::fabs(result.output->front().longitudinal_velocity_mps - 1.0F) < 1.0e-4F &&
            std::fabs(result.output->back().longitudinal_velocity_mps) < 0.01F,
        "초기 속도와 종점의 소프트 속도 제약을 반영해야 합니다");

  planning::Trajectory long_input;
  for (int i = 0; i <= 40; ++i) {
    long_input.push_back({static_cast<double>(i), 0.0,
                          i == 40 ? 0.0F : 5.0F, 0.0F});
  }
  p.resampling.min_trajectory_length = 40.0;
  const auto long_result = planning::runJerkSmoother(long_input, 5.0, 0.0, p);
  check(long_result.output.has_value() && long_result.qp_result.solved,
        "긴 직선 정지 궤적에서도 수렴해야 합니다");

  p.solver.max_iterations = 1;
  const auto unfinished = planning::runJerkSmoother(long_input, 5.0, 0.0, p);
  check(unfinished.output.has_value() && !unfinished.qp_result.solved &&
            unfinished.output->size() == unfinished.optimization_input.size() &&
            unfinished.output->back().longitudinal_velocity_mps == 0.0F,
        "QP 실패 시 원본 apply처럼 재표본화된 경로를 보존합니다");
  const planning::Trajectory terminal{{2.0, 0.0, 0.0F, 0.0F}};
  const auto one_point = planning::runJerkSmoother(terminal, 0.25, 0.0, p);
  check(one_point.output && one_point.output->size() == 1 &&
            !one_point.qp_result.solved &&
            one_point.output->front().longitudinal_velocity_mps == 0.25F,
        "종점 한 점만 남아도 QP 없이 초기 속도 상태를 출력합니다");
  std::cout << "저크 스무더 통합 통과\n";
}
