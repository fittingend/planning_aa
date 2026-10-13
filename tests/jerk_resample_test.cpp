#include "planning/jerk_resample.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <stdexcept>

namespace {
void check(bool condition, const char *message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    std::exit(EXIT_FAILURE);
  }
}
bool near(double a, double b) { return std::fabs(a - b) < 1.0e-5; }
}  // 익명 네임스페이스

int main() {
  planning::JerkResampleParameters p;
  p.max_trajectory_length = 20.0;
  p.min_trajectory_length = 8.0;
  p.resample_time = 0.4;
  p.dense_resample_dt = 0.2;
  p.dense_min_interval_distance = 0.5;
  p.sparse_resample_dt = 0.5;
  p.sparse_min_interval_distance = 2.0;

  const planning::Trajectory stopped{{0.0, 0.0, 2.0F, 0.0F},
                                     {3.0, 0.0, 1.0F, -0.5F},
                                     {4.0, 0.0, 0.0F, -1.0F},
                                     {10.0, 0.0, 0.0F, 0.0F}};
  const auto result = planning::resampleJerkTrajectory(stopped, 2.0, p);
  check(result.size() == 8, "dense, sparse, and stop samples are present");
  check(near(result[1].x, 0.5) && near(result[2].x, 1.0) &&
            near(result[3].x, 3.0) && near(result[4].x, 4.0) &&
            near(result[5].x, 5.0) && near(result[6].x, 7.0) &&
            near(result[7].x, 8.0),
        "source distance schedule inserts the exact stop point");
  check(result[3].longitudinal_velocity_mps == 1.0F &&
            result[4].longitudinal_velocity_mps == 0.0F &&
            result[5].longitudinal_velocity_mps == 0.0F,
        "speed uses zero-order hold and remains zero after stop");
  check(result[1].acceleration_mps2 == 0.0F &&
            result[3].acceleration_mps2 == -0.5F,
        "acceleration uses zero-order hold");

  p.min_trajectory_length = 100.0;
  const auto to_end = planning::resampleJerkTrajectory(stopped, 2.0, p);
  check(near(to_end.back().x, 10.0) && to_end.back().longitudinal_velocity_mps == 0.0F,
        "input endpoint is copied when sampling reaches it");

  bool rejected = false;
  try {
    planning::resampleJerkTrajectory({stopped[0], stopped[0]}, 2.0, p);
  } catch (const std::invalid_argument &) {
    rejected = true;
  }
  check(rejected, "duplicate points cannot enter interpolation");

  planning::TrajectoryPoint ego;
  ego.x = 2.5;
  const auto node_input = planning::resampleJerkNodeInput(
      stopped, ego, 5.0, 1.0, p, 1.0);
  bool has_ego = false;
  bool has_stop = false;
  for (const auto &point : node_input) {
    has_ego |= near(point.x, 2.5);
    has_stop |= near(point.x, 4.0) && point.longitudinal_velocity_mps == 0.0F;
  }
  check(has_ego && has_stop && near(node_input.back().x, 10.0),
        "노드 입력 재샘플링은 차량 투영점, 정지점, 종점을 보존합니다");

  const auto node_output = planning::resampleJerkNodeOutput(
      stopped, 2.0, ego, 5.0, 1.0, p);
  has_ego = false;
  has_stop = false;
  for (const auto &point : node_output) {
    has_ego |= near(point.x, 2.5);
    has_stop |= near(point.x, 4.0) && point.longitudinal_velocity_mps == 0.0F;
  }
  check(has_ego && has_stop && near(node_output.back().x, 10.0),
        "노드 출력 재샘플링은 차량 투영점, 정지점, 종점을 보존합니다");

  const planning::Trajectory stopped_behind{
      {0.0, 0.0, 0.0F, 0.0F}, {3.0, 0.0, 2.0F, 0.0F},
      {6.0, 0.0, 2.0F, 0.0F}, {10.0, 0.0, 0.0F, 0.0F}};
  const auto forward_output = planning::resampleJerkNodeOutput(
      stopped_behind, 2.0, ego, 5.0, 1.0, p);
  const auto moving_ahead = std::find_if(
      forward_output.begin(), forward_output.end(),
      [](const auto &point) { return point.x >= 5.0 && point.x <= 6.0; });
  check(moving_ahead != forward_output.end() &&
            moving_ahead->longitudinal_velocity_mps > 1.0F,
        "차량 뒤쪽의 정지점은 앞쪽 이동 궤적을 0속도로 만들지 않습니다");
  const planning::Trajectory restarted{
      {0.0, 0.0, 2.0F, 0.0F}, {1.0, 0.0, 0.0F, 0.0F},
      {2.0, 0.0, 0.0F, 0.0F}, {3.0, 0.0, 3.0F, 0.0F},
      {10.0, 0.0, 0.0F, 0.0F}};
  ego.x = 0.0;
  const auto source_output = planning::resampleJerkNodeOutput(
      restarted, 2.0, ego, 5.0, 1.0, p);
  const auto recorded_output = planning::resampleJerkNodeOutput(
      restarted, 2.0, ego, 5.0, 1.0, p, true);
  const auto moving_again = [](const planning::Trajectory &trajectory) {
    return std::find_if(trajectory.begin(), trajectory.end(),
                        [](const auto &point) { return near(point.x, 3.0); });
  };
  check(moving_again(source_output) != source_output.end() &&
            moving_again(recorded_output) != recorded_output.end() &&
            moving_again(source_output)->longitudinal_velocity_mps == 0.0F &&
            moving_again(recorded_output)->longitudinal_velocity_mps == 3.0F,
        "기록 호환 재표본화에서만 일시 정지 뒤 속도 회복을 보존합니다");
  std::cout << "jerk resampling passed\n";
}
