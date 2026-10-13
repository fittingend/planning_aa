// SPDX-License-Identifier: Apache-2.0
// 설치한 계산 코어를 다른 C++ 프로젝트에서 호출하는 최소 예시입니다.

#include <planning/behavior_path_prediction.hpp>
#include <planning/behavior_path_object_safety.hpp>
#include <planning/behavior_path_lane_safety.hpp>
#include <planning/behavior_lane_change_safety.hpp>
#include <planning/behavior_static_avoidance.hpp>
#include <planning/behavior_route_action.hpp>
#include <planning/behavior_frenet_sampling.hpp>
#include <planning/behavior_dynamic_avoidance.hpp>
#include <planning/behavior_pull_over_core.hpp>
#include <planning/occlusion_spot_core.hpp>
#include <planning/jerk_smoother.hpp>

#include <cstddef>

int main() {
  planning::Trajectory path(2);
  path[1].x = 10.0;
  planning::BehaviorLaneChangePredictionParameters parameters;
  parameters.initial_velocity_mps = 1.0;
  parameters.prepare_duration_s = 1.0;
  parameters.prepare_velocity_limit_mps = 2.0;
  const auto predicted = planning::predictBehaviorLaneChangeEgoPath(
      path, parameters);
  planning::BehaviorLaneChangeCandidateSafetyInput candidate;
  candidate.prediction = parameters;
  candidate.prediction.lane_change_duration_s = 1.0;
  candidate.prediction.lane_change_velocity_limit_mps = 2.0;
  const auto candidate_safety =
      planning::evaluateBehaviorLaneChangeCandidateSafety(path, candidate);

  planning::Trajectory velocity_path(25);
  for (std::size_t index = 0; index < velocity_path.size(); ++index) {
    velocity_path[index].x = static_cast<double>(index);
    velocity_path[index].longitudinal_velocity_mps = 5.0F;
  }
  velocity_path.back().longitudinal_velocity_mps = 0.0F;
  planning::BehaviorFrenetSamplingInput sampling_input;
  sampling_input.reference = velocity_path;
  sampling_input.target_lengths_m = {10.0};
  sampling_input.target_lateral_positions_m = {0.0};
  sampling_input.target_lateral_derivatives = {0.0};
  const auto sampled = planning::generateBehaviorFrenetCandidates(
      sampling_input);
  planning::BehaviorDynamicAvoidanceTracker dynamic_tracker;
  const auto dynamic_pending = dynamic_tracker.advance(
      {{"차량", 1, 2.0, 10.0, 0.0}});
  const auto smoothed = planning::runJerkSmoother(
      velocity_path, 3.0, 0.0);
  const auto occlusion_path = planning::resampleOcclusionDetectionPath(
      velocity_path, 20.0);
  planning::BehaviorGoalPath route_path;
  route_path.points = velocity_path;
  route_path.lane_ids.assign(velocity_path.size(), {1});
  const auto route_action = planning::composeBehaviorRouteAction(
      route_path, velocity_path.front(),
      {{{-1.0, -2.0}, {25.0, -2.0},
        {25.0, 2.0}, {-1.0, 2.0}}}, {});
  planning::BehaviorSafetyObject object;
  object.length_m = 4.0;
  object.width_m = 2.0;
  const auto object_polygon = planning::behaviorSafetyWorldPolygon(
      {0.0, 10.0, -1.5, 0.0, 0.0}, object);
  planning::StaticAvoidanceMarginInput margin;
  margin.vehicle_width_m = 2.0;
  margin.road_shoulder_distance_m = 0.0;
  margin.lateral_hard_margin_m = 0.2;
  margin.lateral_soft_margin_m = 0.7;
  margin.soft_drivable_bound_margin_m = 0.5;
  margin.hard_drivable_bound_margin_m = 0.1;
  const auto avoidance = planning::staticAvoidanceObjectGeometry(
      velocity_path, 0.0, 0.0, object_polygon,
      velocity_path[10], 0.0, true, margin,
      planning::StaticAvoidanceBoundaryInput{
          {{0.0, 3.0}, {24.0, 3.0}}, 0.0});
  const auto start_samples = planning::sampleStartPullOutBackwardPoses(
      {{0.0, 0.0}, {10.0, 0.0}},
      {0.0, 5.0, 1.0, 0.0, 0.0}, {0.0, 2.0});
  planning::StartPullOutCandidatePlanInput start_input;
  start_input.centerline = {{0.0, 0.0}, {30.0, 0.0}};
  start_input.lane_polygons = {
      {{0.0, -2.0}, {30.0, -2.0}, {30.0, 2.0}, {0.0, 2.0}}};
  start_input.original_start_pose = {0.0, 10.0, 0.0, 0.0, 0.0};
  start_input.start_lane_arc_m = 10.0;
  start_input.lane_length_m = 30.0;
  const auto start_candidates =
      planning::planStartPullOutPoseCandidates(start_input);
  const auto trimmed = planning::trimStaticAvoidanceShiftArcs(
      {{1.0, 3.0, 0.0, 1.0}, {3.0, 5.0, 1.0, 2.0}});
  const auto multi_avoidance = planning::planStaticAvoidanceMultiObject(
      velocity_path, 0, 0.0, 0.0,
      {{5.0, 10.0, 0.0, 1.0},
       {10.0, 15.0, 1.0, 1.0},
       {15.0, 20.0, 1.0, 0.0}});
  const auto bezier_parking = planning::generateBezierPullOverCandidates(
      velocity_path, velocity_path.front(), {30.0, -1.0, 0.0});
  const auto geometric_parking =
      planning::generateGeometricPullOverCandidate(
          velocity_path, velocity_path.front(), {20.0, -1.0, 0.0});
  planning::CrosswalkOccupancyGrid parking_grid;
  parking_grid.width = 60;
  parking_grid.height = 60;
  parking_grid.origin_x = -10.0;
  parking_grid.origin_y = -30.0;
  parking_grid.values.assign(3600, 0);
  const auto freespace_parking =
      planning::generateFreespacePullOverCandidate(
          velocity_path.front(), {12.0, -2.0, 0.0},
          parking_grid);
  planning::FreespacePullOverParameters rrt_parameters;
  rrt_parameters.use_rrt_star = true;
  rrt_parameters.rrt_iterations = 100;
  const auto rrt_parking =
      planning::generateFreespacePullOverCandidate(
          velocity_path.front(), {12.0, -2.0, 0.0},
          parking_grid, rrt_parameters);
  return predicted.size() == 2 && candidate_safety.safe &&
                 sampled.size() == 1 && dynamic_pending.empty() &&
                 !smoothed.stages.merged.empty() &&
                 occlusion_path.size() >= 2 &&
                 route_action.path.points.size() == velocity_path.size() &&
                 avoidance.has_value() && start_samples.size() == 2 &&
                 start_samples[1].has_value() &&
                 !start_candidates.empty() &&
                 !trimmed.empty() && multi_avoidance.has_value() &&
                 bezier_parking.size() == 48 &&
                 geometric_parking.has_value() &&
                 freespace_parking.has_value() &&
                 !freespace_parking->phase_end_indices.empty() &&
                 rrt_parking.has_value() ? 0 : 1;
}
