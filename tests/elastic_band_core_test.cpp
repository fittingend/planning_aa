#include "planning/elastic_band_core.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>

namespace {
void check(bool condition, const char *message) {
  if (!condition) {
    std::cerr << message << '\n';
    std::exit(EXIT_FAILURE);
  }
}

double bendCost(const planning::Trajectory &path) {
  double cost = 0.0;
  for (std::size_t i = 0; i + 2 < path.size(); ++i) {
    const double bend = path[i].y - 2.0 * path[i + 1].y + path[i + 2].y;
    cost += bend * bend;
  }
  return cost;
}
}  // 익명 네임스페이스

int main() {
  planning::Trajectory straight;
  for (int i = 0; i < 12; ++i) {
    straight.push_back({static_cast<double>(i), 0.0, 5.0F, 0.0F});
  }
  const auto straight_result = planning::smoothElasticBandResampled(straight, true);
  check(straight_result && straight_result->trajectory.size() == straight.size(),
        "직선 경로 최적화에 실패했습니다");
  for (std::size_t i = 0; i < straight.size(); ++i) {
    check(std::fabs(straight_result->trajectory[i].x - straight[i].x) < 1.0e-5 &&
              std::fabs(straight_result->trajectory[i].y) < 1.0e-5 &&
              straight_result->trajectory[i].longitudinal_velocity_mps == 5.0F,
          "직선 경로의 위치 또는 속도가 바뀌었습니다");
  }

  auto zigzag = straight;
  for (std::size_t i = 1; i + 2 < zigzag.size(); ++i) {
    zigzag[i].y = i % 2 ? 0.25 : -0.25;
  }
  const auto smoothed = planning::smoothElasticBandResampled(zigzag, true);
  check(smoothed && smoothed->optimized_points == zigzag.size(),
        "굴곡 경로 최적화에 실패했습니다");
  check(bendCost(smoothed->trajectory) < bendCost(zigzag),
        "굴곡 경로의 평활화 비용이 줄지 않았습니다");
  check(std::fabs(smoothed->trajectory.front().y - zigzag.front().y) < 1.0e-5 &&
            std::fabs(smoothed->trajectory[10].y - zigzag[10].y) < 1.0e-5 &&
            std::fabs(smoothed->trajectory[11].y - zigzag[11].y) < 1.0e-5,
        "고정된 첫 점과 목표점 두 점이 움직였습니다");
  for (std::size_t i = 0; i < zigzag.size(); ++i) {
    const double clearance = i == 0 || i >= 10 ? 0.0 : 0.1;
    check(std::fabs(smoothed->trajectory[i].y - zigzag[i].y) <=
              clearance + 1.0e-5,
          "횡방향 이동 제한을 넘었습니다");
  }
  planning::Trajectory long_path;
  for (int i = 0; i < 150; ++i) {
    long_path.push_back({static_cast<double>(i), 0.0, 5.0F, 0.0F});
  }
  planning::TrajectoryPoint ego;
  ego.x = 20.0;
  const auto prepared = planning::prepareElasticBandInput(long_path, ego);
  check(!prepared.goal_contained &&
            std::fabs(prepared.trajectory.front().x - 15.0) < 1.0e-5 &&
            std::fabs(prepared.trajectory.back().x - 120.0) < 1.0e-5 &&
            prepared.trajectory.size() == 106,
        "차량 주변 경로 자르기와 1m 재표본화가 다릅니다");
  planning::Trajectory previous;
  for (int i = 14; i < 114; ++i) {
    previous.push_back({static_cast<double>(i), 0.0, 5.0F, 0.0F});
  }
  const auto prepared_with_previous = planning::prepareElasticBandInput(
      long_path, ego, &previous);
  check(std::fabs(prepared_with_previous.trajectory.front().x - 14.0) < 1.0e-5 &&
            prepared_with_previous.trajectory.size() == 107,
        "이전 평활화 경로의 첫 고정점이 이어지지 않았습니다");
  const auto prepared_result = planning::smoothElasticBandResampled(
      prepared.trajectory, prepared.goal_contained);
  check(prepared_result && prepared_result->trajectory.size() == 100,
        "재표본화한 경로의 100점 최적화에 실패했습니다");
  const auto finalized = planning::finalizeElasticBandTrajectory(
      long_path, prepared_result->trajectory, ego);
  check(std::fabs(finalized.front().x - 15.0) < 1.0e-5 &&
            std::fabs(finalized.back().x - 149.0) < 1.0e-5 &&
            finalized.size() > prepared_result->trajectory.size(),
        "최적화 경로 뒤에 남은 입력 경로가 연결되지 않았습니다");
  for (std::size_t i = 30; i < long_path.size(); ++i) {
    long_path[i].longitudinal_velocity_mps = 0.0F;
  }
  const auto stopped_output = planning::finalizeElasticBandTrajectory(
      long_path, prepared_result->trajectory, ego);
  bool stop_found = false;
  for (const auto &point : stopped_output) {
    if (std::fabs(point.x - 30.0) < 1.0e-5) {
      stop_found = point.longitudinal_velocity_mps == 0.0F;
    }
    if (point.x >= 30.0) {
      check(point.longitudinal_velocity_mps == 0.0F,
            "정지점 이후의 속도가 0이 아닙니다");
    }
  }
  check(stop_found, "입력 경로의 정지점이 출력에 보존되지 않았습니다");
  planning::ElasticBandPlanner planner;
  const auto first_plan = planner.run(long_path, ego, 5.0, 1.0);
  check(first_plan.reset && first_plan.replanned &&
            !first_plan.optimized.empty(),
        "첫 입력에서 평활화 상태가 생성되지 않았습니다");
  const auto same_time_plan = planner.run(long_path, ego, 5.0, 1.0);
  check(!same_time_plan.reset && !same_time_plan.replanned,
        "같은 시각의 동일 경로를 불필요하게 다시 계산했습니다");
  const auto next_plan = planner.run(long_path, ego, 5.0, 1.1);
  check(!next_plan.reset && next_plan.replanned,
        "다음 시각의 경로를 재계획하지 않았습니다");
  ego.x = 30.0;
  const auto moved_plan = planner.run(long_path, ego, 5.0, 1.2);
  check(moved_plan.reset && moved_plan.replanned,
        "차량 위치가 5m보다 멀리 바뀌었는데 상태를 초기화하지 않았습니다");
  std::cout << "Elastic Band 계산 검사 통과\n";
}
