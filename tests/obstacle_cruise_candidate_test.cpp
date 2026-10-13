// SPDX-License-Identifier: Apache-2.0
// 순항 후보의 원본 제외 조건 경계를 검사합니다.

#include "planning/obstacle_cruise_candidate.hpp"

#include <cmath>
#include <stdexcept>

namespace {
void check(bool condition) {
  if (!condition) throw std::runtime_error("순항 후보 검사 실패");
}
}  // 익명 네임스페이스

int main() {
  using planning::CruiseCandidateRejection;
  const std::vector<planning::CruisePredictedPath> predictions{
      {{{0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}}, 1.0, 0.2},
      {{{0.0, 0.0, 0.0}, {0.0, 10.0, std::acos(-1.0) / 2.0}}, 1.0, 0.9}};
  const auto current = planning::currentCruiseObjectPose(
      {5.0, 5.0, 0.0}, predictions, 0.5);
  check(std::abs(current.x) < 1e-12 &&
        std::abs(current.y - 5.0) < 1e-12 &&
        std::abs(current.yaw_rad - std::acos(-1.0) / 4.0) < 1e-12);
  const auto fallback = planning::currentCruiseObjectPose(
      {5.0, 5.0, 0.0}, predictions, 2.0);
  check(fallback.x == 5.0 && fallback.y == 5.0);
  planning::Trajectory path(11);
  for (std::size_t i = 0; i < path.size(); ++i)
    path[i].x = static_cast<double>(i);
  const auto projection = planning::projectCruiseObjectToTrajectory(
      path, 2, {6.0, 2.0, 0.0, 4.0, -1.0});
  check(projection && std::abs(projection->distance_from_ego_m - 4.0) < 1e-12 &&
        std::abs(projection->signed_lateral_offset_m - 2.0) < 1e-12 &&
        std::abs(projection->longitudinal_velocity_mps - 4.0) < 1e-12 &&
        std::abs(projection->approach_velocity_mps - 1.0) < 1e-12);
  check(!planning::projectCruiseObjectToTrajectory(
      path, 2, {1.0, 0.0, 0.0, 1.0, 0.0}));
  const auto crossing_projection = planning::projectCruiseObjectToTrajectory(
      path, 2, {6.0, -2.0, std::acos(-1.0) / 2.0, 3.0, 0.0});
  check(crossing_projection &&
        std::abs(crossing_projection->approach_velocity_mps - 3.0) < 1e-12);
  check(planning::passesObstacleCruiseRoughLateralFilter(
      *projection, 1.0, 1.83));
  auto far_projection = *projection;
  far_projection.signed_lateral_offset_m = 9.0;
  check(!planning::passesObstacleCruiseRoughLateralFilter(
      far_projection, 1.0, 1.83));
  far_projection.approach_velocity_mps = 2.0;
  check(planning::passesObstacleCruiseRoughLateralFilter(
      far_projection, 1.0, 1.83));
  planning::CruiseCandidateInput input;
  input.supported_class = true;
  input.outside_class = true;
  input.ego_speed_mps = 5.0;
  input.longitudinal_speed_mps = 4.0;
  input.approach_speed_mps = 1.0;
  input.has_collision_points = true;
  input.collision_overlap_s = 3.0;
  input.object_speed_mps = 4.0;
  const auto classify = [&]() {
    return planning::classifyCruiseCandidate(input);
  };
  check(classify() == CruiseCandidateRejection::None);
  input.longitudinal_speed_mps = -0.1;
  check(classify() == CruiseCandidateRejection::OppositeDirection);
  input.longitudinal_speed_mps = 4.0;
  input.lateral_distance_m = 1.01;
  check(classify() == CruiseCandidateRejection::None);
  input.lateral_distance_m = 6.0;
  check(classify() == CruiseCandidateRejection::TooFarLaterally);
  input.lateral_distance_m = 0.5;
  input.heading_difference_rad = 1.5;
  check(classify() == CruiseCandidateRejection::CrossingTrajectory);
  input.heading_difference_rad = 0.0;
  input.ego_speed_mps = 0.09;
  check(classify() == CruiseCandidateRejection::OutsideWhileEgoStopped);
  input.ego_speed_mps = 5.0;
  input.object_speed_mps = 1.49;
  check(classify() == CruiseCandidateRejection::OutsideTooSlow);
  input.object_speed_mps = 4.0;
  input.collision_overlap_s = 0.19;
  check(classify() == CruiseCandidateRejection::OutsideShortOverlap);
  input.collision_overlap_s = 0.2;
  input.first_collision_in_front = false;
  check(classify() == CruiseCandidateRejection::OutsideBehindEgo);
  input.first_collision_in_front = true;
  check(classify() == CruiseCandidateRejection::None);
  input.lateral_distance_m = 0.0;
  input.ego_speed_mps = 0.0;
  input.object_speed_mps = 0.0;
  input.collision_overlap_s = 0.0;
  check(classify() == CruiseCandidateRejection::None);

  planning::CruiseInsideHysteresis hysteresis;
  check(!hysteresis.permits("객체A", 1.49));
  check(hysteresis.permits("객체A", 1.5));
  hysteresis.replacePreviousCruiseIds({"객체A"});
  check(hysteresis.permits("객체A", 1.0));
  check(!hysteresis.permits("객체A", 0.99));
  check(!hysteresis.permits("객체B", 1.0));
  hysteresis.replacePreviousCruiseIds({"객체B"});
  check(!hysteresis.permits("객체A", 1.0));
  check(hysteresis.permits("객체B", 1.0));
  hysteresis.reset();
  check(!hysteresis.permits("객체B", 1.0));
  bool rejected = false;
  try { planning::CruiseInsideHysteresis invalid(1.0, 1.5); }
  catch (const std::invalid_argument &) { rejected = true; }
  check(rejected);

  std::vector<planning::CruiseYieldObject> yield_objects{
      {"이동", true, false, 10.0, 2.0, 2.0, 0.0,
       2.0, 2.0, 10.0},
      {"정차", false, false, 14.0, 3.0, 0.0, 0.0,
       0.0, 3.0, 14.0},
      {"원거리", false, false, 40.0, 3.0, 0.0, 0.0,
       0.0, 3.0, 40.0}};
  auto pairs = planning::findCruiseYieldPairs(yield_objects);
  check(pairs.size() == 1 && pairs.front().moving_index == 0 &&
        pairs.front().stopped_index == 1);
  yield_objects[0].crossing_trajectory = true;
  check(planning::findCruiseYieldPairs(yield_objects).empty());
  yield_objects[0].crossing_trajectory = false;
  yield_objects[0].distance_from_ego_m = 4.77;
  check(planning::findCruiseYieldPairs(yield_objects).empty());
  yield_objects[0].distance_from_ego_m = 10.0;
  yield_objects[0].local_velocity_x_mps = 1.49;
  check(planning::findCruiseYieldPairs(yield_objects).empty());
}
