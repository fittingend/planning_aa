// 원본 돌발 진입 상태 기계의 경계 시각과 장애물 유지 동작을 검사합니다.

#include "planning/run_out_state.hpp"

#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <cmath>
#include <cstdint>
#include <optional>

int main() {
  using planning::RunOutState;
  using planning::RunOutStateCore;
  using planning::RunOutStateInput;
  RunOutStateCore core;
  auto step = [&](double time, double velocity, double distance,
                  std::optional<std::uint64_t> obstacle) {
    return core.update(RunOutStateInput{time, velocity, distance, obstacle});
  };

  assert(step(0.0, 2.0, 0.5, 7).state == RunOutState::Go);
  assert(step(0.1, 1.0, 0.5, 7).state == RunOutState::Stop);
  assert(step(1.1, 1.0, 0.5, 7).state == RunOutState::Stop);
  assert(step(1.101, 1.0, 0.5, 7).state == RunOutState::Approach);
  auto held = step(1.5, 1.0, 2.0, std::nullopt);
  assert(held.state == RunOutState::Approach);
  assert(held.target_obstacle_id == 7);
  held = step(2.101, 1.0, 2.0, std::nullopt);
  assert(held.state == RunOutState::Go);
  assert(!held.target_obstacle_id);

  assert(step(2.2, 1.0, 0.5, 8).state == RunOutState::Stop);
  assert(step(2.3, 1.0, 0.5, std::nullopt).state == RunOutState::Go);
  assert(step(2.4, 1.0, 0.5, 8).state == RunOutState::Stop);
  assert(step(3.5, 1.0, 0.5, 8).state == RunOutState::Approach);
  assert(step(3.6, 1.0, 0.5, 9).target_obstacle_id == 9);
  held = step(3.7, 1.0, 2.0, std::nullopt);
  assert(held.state == RunOutState::Approach);
  assert(held.target_obstacle_id == 9);

  planning::Trajectory path(11);
  for (std::size_t i = 0; i < path.size(); ++i) {
    path[i].x = static_cast<double>(i);
    path[i].longitudinal_velocity_mps = 5.0F;
  }
  path[6].orientation_z = std::sqrt(0.5);
  path[6].orientation_w = std::sqrt(0.5);
  const auto approach = planning::applyRunOutApproachVelocity(
      path, 2, 10.0, {1.5, 1.0, 3.5});
  assert(approach.stop_arc_m == 5.5);
  assert(approach.trajectory.size() == 12);
  assert(approach.trajectory[1].longitudinal_velocity_mps == 5.0F);
  assert(approach.trajectory[2].longitudinal_velocity_mps == 1.5F);
  assert(approach.trajectory[6].x == 5.5);
  assert(std::abs(approach.trajectory[6].orientation_z -
                  std::sin(3.141592653589793 / 8.0)) < 1.0e-6);
  assert(approach.trajectory[6].longitudinal_velocity_mps == 0.0F);
  assert(approach.trajectory.back().longitudinal_velocity_mps == 0.0F);
  const auto outside = planning::applyRunOutApproachVelocity(
      path, 2, 20.0, {1.5, 1.0, 3.5});
  assert(!outside.stop_arc_m);
  assert(outside.trajectory[2].longitudinal_velocity_mps == 1.5F);

  const auto direct_stop = planning::applyRunOutStopVelocity(
      path, 10.0, 1.0, 3.5);
  assert(direct_stop.stop_arc_m == 5.5);
  assert(direct_stop.trajectory.size() == 12);
  assert(direct_stop.trajectory[6].x == 5.5);
  assert(direct_stop.trajectory[5].longitudinal_velocity_mps == 5.0F);
  assert(direct_stop.trajectory[6].longitudinal_velocity_mps == 0.0F);
  const auto too_close = planning::applyRunOutStopVelocity(
      path, 3.0, 1.0, 3.5);
  assert(!too_close.stop_arc_m);
  assert(too_close.trajectory.size() == path.size());
  assert(too_close.trajectory.back().longitudinal_velocity_mps == 5.0F);

  const auto near_braking = planning::planRunOutBraking(
      path, {8.0, 0.0, 5.0, 0.0, true});
  assert(near_braking.distance_valid);
  assert(near_braking.deceleration_required);
  assert(near_braking.stop_arc_m &&
         std::abs(*near_braking.stop_arc_m - 1.71) < 1.0e-9);
  assert(near_braking.trajectory.back().longitudinal_velocity_mps == 0.0F);
  const auto far_braking = planning::planRunOutBraking(
      path, {200.0, 0.0, 5.0, 0.0, true});
  assert(!far_braking.deceleration_required);
  assert(!far_braking.stop_arc_m);
  const auto point_detection = planning::planRunOutBraking(
      path, {8.0, 0.0, 0.5, -0.1, false});
  assert(point_detection.stop_arc_m &&
         std::abs(*point_detection.stop_arc_m - 1.71) < 1.0e-9);
  planning::RunOutBrakingParameters source_distance_parameters;
  source_distance_parameters.deceleration_jerk_mps3 = -0.5;
  source_distance_parameters.normal_min_jerk_mps3 = -0.5;
  source_distance_parameters.normal_min_acceleration_mps2 = -0.5;
  const auto ordinary_distance = planning::planRunOutBraking(
      path, {400.0, 0.0, 16.7, 0.0, true},
      source_distance_parameters);
  assert(ordinary_distance.distance_valid &&
         std::abs(ordinary_distance.required_stop_distance_m - 287.240) < 0.001);
  const auto already_braking = planning::planRunOutBraking(
      path, {400.0, 0.0, 16.7, -2.5, true},
      source_distance_parameters);
  assert(already_braking.distance_valid &&
         std::abs(already_braking.required_stop_distance_m - 166.6441667) < 0.001);

  planning::RunOutTemporalCore temporal;
  assert(!temporal.acceptCollision(0.0, true));
  assert(!temporal.acceptCollision(0.149, true));
  assert(temporal.acceptCollision(0.15, true));
  assert(!temporal.acceptCollision(0.2, false));
  assert(!temporal.acceptCollision(0.3, true));
  temporal.recordStop(1.0, {5.5, 0.0});
  assert(!temporal.maintainedStop(1.1, true));
  assert(temporal.maintainedStop(2.999, false)->x == 5.5);
  assert(!temporal.maintainedStop(3.0, false));
  planning::RunOutTemporalCore immediate({false, 0.15, 2.0, false});
  assert(immediate.acceptCollision(0.0, true));
  planning::RunOutTemporalCore approaching({true, 0.15, 2.0, true});
  approaching.recordStop(1.0, {5.5, 0.0});
  assert(!approaching.maintainedStop(1.1, false));
}
