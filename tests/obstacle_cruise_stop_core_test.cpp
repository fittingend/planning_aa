// SPDX-License-Identifier: Apache-2.0
// 장애물별 최근접 후보, 감속 거리, 정지 유지와 점 삽입을 검사합니다.

#include "planning/obstacle_cruise_stop_core.hpp"

#include <cmath>
#include <stdexcept>

namespace {
void check(bool condition) {
  if (!condition) throw std::runtime_error("장애물 순항 정지 검사 실패");
}
}  // 네임스페이스

int main() {
  using planning::CruiseStopRejection;
  planning::CruiseStopBehaviorInput object;
  object.supported_class = true;
  object.inside_class = true;
  object.has_collision_point = true;
  object.time_to_reach_stop_point_s = 2.0;
  object.checked_future_collision = true;
  check(planning::classifyCruiseStopBehavior(object) ==
        CruiseStopRejection::TransientInsideObject);
  planning::CruiseStopBehaviorParameters library_parameters;
  library_parameters.collision_time_margin_s = 4.0;
  library_parameters.maximum_lateral_time_s = 4.0;
  check(planning::classifyCruiseStopBehavior(
            object, library_parameters) == CruiseStopRejection::None);
  object.has_future_collision = true;
  check(planning::classifyCruiseStopBehavior(object) ==
        CruiseStopRejection::None);
  object.precise_lateral_distance_m = 0.31;
  object.outside_class = true;
  object.approach_velocity_mps = 0.05;
  check(planning::classifyCruiseStopBehavior(object) ==
        CruiseStopRejection::OutsideTooFar);
  object.approach_velocity_mps = 0.1;
  object.has_predicted_path = true;
  object.collision_time_gap_s = 1.01;
  check(planning::classifyCruiseStopBehavior(object) ==
        CruiseStopRejection::OutsideTimeGap);
  object.collision_time_gap_s = 1.0;
  check(planning::classifyCruiseStopBehavior(object) ==
        CruiseStopRejection::None);
  check(std::abs(planning::cruiseStopTimeToReachCollision(
      10.0, 4.0, 1.0) - 3.0) < 1e-12);
  check(planning::cruiseStopCollisionTimeGap(
      3.0, 4.0, 2.0, 6.0, 7.0) == 3.0);
  check(planning::cruiseStopCollisionTimeGap(
      3.0, 4.0, 2.0, 0.0, 2.0) == 1.0);
  check(planning::cruiseStopCollisionTimeGap(
      3.0, 4.0, 2.0, 4.0, 6.0) == 0.0);
  check(planning::chooseCruiseStopMargin(
      30.0, 100.0, 27.0, 6.0, 3.0, 3.0) == 3.0);
  check(planning::chooseCruiseStopMargin(
      30.0, 100.0, 24.0, 6.0, 3.0, 3.0) == 6.0);
  check(planning::chooseCruiseStopMargin(
      101.0, 100.0, 102.0, 6.0, 3.0, 3.0) == 3.0);
  planning::Trajectory path(11);
  for (std::size_t i = 0; i < path.size(); ++i) {
    path[i].x = i * 2.0;
    path[i].longitudinal_velocity_mps = 5.0F;
  }
  planning::ObstacleCruiseStopCore core;
  const auto first = core.plan(path, 1.0, 5.0,
      {{1, 15.0, 3.0}, {1, 14.0, 3.0}, {2, 13.0, 4.0}});
  check(first.stop_distance_from_start.has_value());
  check(std::abs(*first.stop_distance_from_start - 9.0) < 1e-12);
  check(first.classification == 2);
  check(first.stop_index == 5);
  check(first.trajectory.size() == path.size() + 1);
  check(first.trajectory[5].longitudinal_velocity_mps == 0.0F);
  check(first.trajectory[4].longitudinal_velocity_mps == 5.0F);
  planning::Trajectory shifted(path.begin() + 1, path.end());
  const auto held = core.plan(shifted, 0.0, 0.0, {{2, 11.2, 4.0}});
  check(held.stop_distance_from_start.has_value());
  check(std::abs(*held.stop_distance_from_start - 7.0) < 1e-12);
  planning::ObstacleCruiseStopParameters braking;
  braking.suppress_sudden_stop = true;
  planning::ObstacleCruiseStopCore braking_core(braking);
  const auto limited = braking_core.plan(path, 0.0, 6.0, {{1, 7.0, 1.0}});
  check(limited.stop_distance_from_start.has_value());
  check(std::abs(*limited.stop_distance_from_start - 6.0) < 1e-12);
  const auto abandoned = braking_core.plan(
      path, 0.0, 6.0, {{1, 5.0, 1.0, -2.0, true, true}});
  check(!abandoned.stop_distance_from_start.has_value());
  const auto special = braking_core.plan(
      path, 0.0, 6.0, {{1, 7.0, 1.0, -2.0, false, true}});
  check(special.stop_distance_from_start.has_value());
  check(std::abs(*special.stop_distance_from_start - 9.0) < 1e-12);
  const auto beyond_threshold = braking_core.plan(
      path, 0.0, 6.0, {{1, 19.0, 1.0, -2.0, false, true}});
  check(beyond_threshold.stop_distance_from_start.has_value());
  check(std::abs(*beyond_threshold.stop_distance_from_start - 18.0) < 1e-12);
  const auto distance_threshold = braking_core.plan(
      path, 0.0, 6.0, {{1, 10.0, 1.0, -2.0, false, true, -1.1, 8.0}});
  check(distance_threshold.stop_distance_from_start.has_value());
  check(std::abs(*distance_threshold.stop_distance_from_start - 9.0) < 1e-12);
  const auto clear = core.plan(path, 1.0, 0.0, {});
  check(!clear.stop_distance_from_start.has_value());
  planning::CruiseStopConsistency consistency;
  const planning::CruiseStopTrackedCandidate tracked{"차량", 1, 10.0, 8.0};
  check(consistency.update(10.0, {{"차량", 0.0}}, {tracked}).size() == 1);
  check(consistency.update(10.5, {{"차량", 1.4}}, {}).size() == 1);
  check(consistency.update(11.0, {{"차량", 1.4}}, {}).empty());
  check(consistency.update(11.1, {{"차량", 0.0}}, {tracked}).size() == 1);
  check(consistency.update(11.2, {}, {}).empty());
  const planning::CruiseStopTrackedCandidate recent{
      "차량", 1, 11.3, 8.0};
  check(consistency.update(11.3, {{"차량", 0.0}}, {recent}).size() == 1);
  check(consistency.update(11.4, {{"차량", 1.4}}, {}).size() == 1);
  check(consistency.update(11.5, {{"차량", 1.5}}, {}).empty());
}
