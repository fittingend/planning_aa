#include "planning/behavior_candidate_callback.hpp"

#include <cstdlib>
#include <iostream>
#include <stdexcept>

int main() {
  planning::BehaviorCandidateCallbackInput input;
  input.callback_time_ns = 2'000'000'000;
  input.maximum_perception_age_ns = 100'000'000;
  input.perception.receipt_time_ns = 1'950'000'000;
  input.start.centerline = {{0.0, 0.0}, {30.0, 0.0}};
  input.start.lane_polygons = {
      {{0.0, -2.0}, {30.0, -2.0}, {30.0, 2.0}, {0.0, 2.0}}};
  input.start.original_start_pose = {0.0, 10.0, 0.0, 0.0, 0.0};
  input.start.start_lane_arc_m = 10.0;
  input.start.lane_length_m = 30.0;
  input.start.base_link_to_rear_m = 1.0;
  input.start.maximum_backward_distance_m = 6.0;
  input.start.backward_search_resolution_m = 2.0;
  input.start.first_collision_margin_m = 0.0;
  input.goal.goal = {20.0, 0.0, 0.0};
  input.goal.object_hard_margin_m = 0.1;
  planning::StartPullOutStopObject object;
  object.pose = {0.0, 20.0, 0.0, 0.0, 0.0};
  object.shape.length_m = 2.0;
  object.shape.width_m = 1.0;
  input.perception.objects = {object};
  const auto near_goal = planning::evaluateBehaviorCandidateCallback(input);
  if (near_goal.start_candidates.empty() ||
      near_goal.goal_safety !=
          planning::BehaviorGoalSafetyReason::FootprintObjectCollision ||
      near_goal.perception_object_count != 1) return EXIT_FAILURE;
  input.perception.objects.front().pose.x = 8.0;
  const auto near_start = planning::evaluateBehaviorCandidateCallback(input);
  if (near_start.start_candidates.size() >= near_goal.start_candidates.size() ||
      near_start.goal_safety != planning::BehaviorGoalSafetyReason::Safe)
    return EXIT_FAILURE;
  input.perception.receipt_time_ns = 1'000'000'000;
  try {
    (void)planning::evaluateBehaviorCandidateCallback(input);
    return EXIT_FAILURE;
  } catch (const std::invalid_argument &) {
    return EXIT_SUCCESS;
  }
}
