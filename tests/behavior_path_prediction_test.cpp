#include "planning/behavior_path_prediction.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>

namespace {

void check(bool condition, const char *message) {
  if (!condition) {
    std::cerr << "검증 실패: " << message << '\n';
    std::exit(EXIT_FAILURE);
  }
}

bool close(double first, double second) {
  return std::fabs(first - second) < 1.0e-9;
}

}  // 익명 네임스페이스

int main() {
  planning::Trajectory path(2);
  path[1].x = 100.0;
  planning::BehaviorLaneChangePredictionParameters parameters;
  parameters.ego_arc_m = 5.0;
  parameters.initial_velocity_mps = 10.0;
  parameters.prepare_duration_s = 2.0;
  parameters.lane_change_duration_s = 2.0;
  parameters.prepare_acceleration_mps2 = 1.0;
  parameters.lane_change_acceleration_mps2 = -2.0;
  parameters.prepare_velocity_limit_mps = 15.0;
  parameters.lane_change_velocity_limit_mps = 15.0;
  const auto result = planning::predictBehaviorLaneChangeEgoPath(
      path, parameters);
  check(result.size() == 8, "준비·변경 시간의 0.5초 표본 수");
  check(close(result[0].time_s, 0.0) &&
            close(result[0].x, 5.0) &&
            close(result[0].velocity_mps, 10.0),
        "현재 호길이와 속도를 첫 표본으로 사용합니다");
  check(close(result[2].time_s, 1.0) &&
            close(result[2].x, 15.5) &&
            close(result[2].velocity_mps, 11.0),
        "준비 구간 등가속도 거리와 속도");
  check(close(result[4].time_s, 2.0) &&
            close(result[4].x, 27.0) &&
            close(result[4].velocity_mps, 12.0),
        "구간 경계의 속도와 누적 거리");
  check(close(result[6].time_s, 3.0) &&
            close(result[6].x, 38.0) &&
            close(result[6].velocity_mps, 10.0),
        "차로 변경 구간 감속 거리와 속도");
  const auto sampled = planning::predictBehaviorLaneChangeEgoPaths(
      path, parameters, -4.0, 4);
  check(sampled.size() == 4 &&
            close(sampled[0][6].x, 38.0) &&
            close(sampled[1][6].x, 37.75) &&
            close(sampled[2][6].x, 37.5) &&
            close(sampled[3][6].x, 37.25),
        "원본 감속 표본은 현재 가속도부터 최소 가속도 직전까지 생성합니다");
  const auto one_path = planning::predictBehaviorLaneChangeEgoPaths(
      path, parameters, -1.0, 4);
  check(one_path.size() == 1 && close(one_path[0][6].x, 38.0),
        "최소 가속도가 추가 감속을 요구하지 않으면 한 경로만 생성합니다");
  parameters.prepare_velocity_limit_mps = 10.0;
  const auto limited = planning::predictBehaviorLaneChangeEgoPath(
      path, parameters);
  check(close(limited[2].velocity_mps, 10.0) &&
            close(limited[2].x, 15.5) &&
            close(limited[4].velocity_mps, 10.0),
        "원본처럼 출력 속도만 제한하고 적분 거리는 별도 계산합니다");
  return EXIT_SUCCESS;
}
