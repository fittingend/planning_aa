// SPDX-License-Identifier: Apache-2.0
// 지도 플러그인 입력을 같은 계획 프레임의 경로·자차 상태와 결합합니다.

#include "planning/planning_pipeline_map_adapter.hpp"
#include "planning/crosswalk_core.hpp"
#include "planning/dynamic_obstacle_stop_core.hpp"
#include "planning/obstacle_cruise_collision.hpp"
#include "planning/jerk_resample.hpp"

#include <lanelet2_core/geometry/LineString.h>

#include <utility>
#include <algorithm>
#include <cmath>
#include <limits>
#include <iterator>
#include <tuple>
#include <string>
#include <stdexcept>
#include <unordered_set>

namespace {
bool sameStartPlan(
    const planning::BehaviorRouteStartRequest &a,
    const planning::BehaviorRouteStartRequest &b) {
  if (a.freespace_grid.has_value() != b.freespace_grid.has_value())
    return false;
  if (a.freespace_grid) {
    const auto &x = *a.freespace_grid;
    const auto &y = *b.freespace_grid;
    if (std::tie(x.width, x.height, x.resolution_m, x.origin_x,
                 x.origin_y, x.origin_yaw_rad, x.values) !=
        std::tie(y.width, y.height, y.resolution_m, y.origin_x,
                 y.origin_y, y.origin_yaw_rad, y.values)) return false;
  }
  const auto &af = a.freespace;
  const auto &bf = b.freespace;
  const auto &ag = a.geometric;
  const auto &bg = b.geometric;
  const auto &ap = a.parameters;
  const auto &bp = b.parameters;
  const auto &av = a.vehicle;
  const auto &bv = b.vehicle;
  if (a.use_ego_pose != b.use_ego_pose ||
      (!a.use_ego_pose &&
       (a.start.x != b.start.x || a.start.y != b.start.y ||
        a.lateral_offset_m != b.lateral_offset_m))) return false;
  return std::tie(a.use_geometric, a.use_geometric_fallback,
                  a.use_freespace, a.use_freespace_fallback,
                  a.object_types_to_check,
                  a.freespace_join_distance_m,
                  a.terminal_is_goal, av.front_m, av.back_m,
                  av.half_width_m, av.sample_spacing_m) ==
             std::tie(b.use_geometric, b.use_geometric_fallback,
                      b.use_freespace, b.use_freespace_fallback,
                      b.object_types_to_check,
                      b.freespace_join_distance_m,
                      b.terminal_is_goal, bv.front_m, bv.back_m,
                      bv.half_width_m, bv.sample_spacing_m) &&
         std::tie(ag.wheelbase_m, ag.maximum_steering_angle_rad,
                  ag.far_turn_radius_m, ag.arc_interval_m,
                  ag.after_straight_distance_m, ag.forward_velocity_mps,
                  ag.backward_velocity_mps, ag.forward) ==
             std::tie(bg.wheelbase_m, bg.maximum_steering_angle_rad,
                      bg.far_turn_radius_m, bg.arc_interval_m,
                      bg.after_straight_distance_m, bg.forward_velocity_mps,
                      bg.backward_velocity_mps, bg.forward) &&
         std::tie(ap.lateral_jerk_mps3,
                  ap.minimum_lateral_acceleration_mps2,
                  ap.maximum_lateral_acceleration_mps2,
                  ap.lateral_acceleration_samples,
                  ap.maximum_curvature,
                  ap.end_curvature_threshold,
                  ap.minimum_distance_m,
                  ap.centerline_interval_m) ==
             std::tie(bp.lateral_jerk_mps3,
                      bp.minimum_lateral_acceleration_mps2,
                      bp.maximum_lateral_acceleration_mps2,
                      bp.lateral_acceleration_samples,
                      bp.maximum_curvature,
                      bp.end_curvature_threshold,
                      bp.minimum_distance_m,
                      bp.centerline_interval_m) &&
         std::tie(af.wheelbase_m, af.maximum_steering_angle_rad,
                  af.expansion_distance_m, af.velocity_mps, af.front_m,
                  af.rear_m, af.half_width_m, af.vehicle_shape_margin_m,
                  af.goal_position_tolerance_m,
                  af.goal_yaw_tolerance_rad,
                  af.final_straight_distance_m,
                  af.use_backward_goal_approach, af.reverse_weight,
                  af.direction_change_weight, af.theta_bins,
                  af.obstacle_threshold, af.maximum_expansions,
                  af.use_reverse, af.use_reeds_shepp_heuristic,
                  af.use_obstacle_distance_heuristic, af.use_rrt_star,
                  af.rrt_iterations, af.rrt_step_m, af.rrt_time_limit_ms,
                  af.rrt_seed) ==
             std::tie(bf.wheelbase_m, bf.maximum_steering_angle_rad,
                      bf.expansion_distance_m, bf.velocity_mps, bf.front_m,
                      bf.rear_m, bf.half_width_m, bf.vehicle_shape_margin_m,
                      bf.goal_position_tolerance_m,
                      bf.goal_yaw_tolerance_rad,
                      bf.final_straight_distance_m,
                      bf.use_backward_goal_approach, bf.reverse_weight,
                      bf.direction_change_weight, bf.theta_bins,
                      bf.obstacle_threshold, bf.maximum_expansions,
                      bf.use_reverse, bf.use_reeds_shepp_heuristic,
                      bf.use_obstacle_distance_heuristic, bf.use_rrt_star,
                      bf.rrt_iterations, bf.rrt_step_m, bf.rrt_time_limit_ms,
                      bf.rrt_seed);
}

bool samePullOverPlan(
    const planning::BehaviorRoutePullOverRequest &a,
    const planning::BehaviorRoutePullOverRequest &b) {
  const auto &ag = a.geometric;
  const auto &bg = b.geometric;
  const auto &af = a.freespace;
  const auto &bf = b.freespace;
  const auto &ap = a.parameters;
  const auto &bp = b.parameters;
  const auto &av = a.vehicle;
  const auto &bv = b.vehicle;
  const auto grid_matches = [&] {
    if (a.freespace_grid.has_value() != b.freespace_grid.has_value())
      return false;
    if (!a.freespace_grid) return true;
    const auto &x = *a.freespace_grid;
    const auto &y = *b.freespace_grid;
    return std::tie(x.width, x.height, x.resolution_m, x.origin_x,
                    x.origin_y, x.origin_yaw_rad, x.values) ==
           std::tie(y.width, y.height, y.resolution_m, y.origin_x,
                    y.origin_y, y.origin_yaw_rad, y.values);
  };
  return std::tie(a.goal.x, a.goal.y, a.goal.yaw_rad,
                  a.goal_lanelet_ids, a.use_bezier, a.use_bezier_fallback,
                  a.use_geometric, a.use_geometric_fallback,
                  a.use_freespace, a.use_freespace_fallback,
                  a.maximum_deceleration_mps2,
                  a.maximum_extra_stopping_margin_m) ==
             std::tie(b.goal.x, b.goal.y, b.goal.yaw_rad,
                      b.goal_lanelet_ids, b.use_bezier,
                      b.use_bezier_fallback, b.use_geometric,
                      b.use_geometric_fallback, b.use_freespace,
                      b.use_freespace_fallback,
                      b.maximum_deceleration_mps2,
                      b.maximum_extra_stopping_margin_m) &&
         std::tie(ag.wheelbase_m, ag.maximum_steering_angle_rad,
                  ag.far_turn_radius_m, ag.arc_interval_m,
                  ag.after_straight_distance_m, ag.forward_velocity_mps,
                  ag.backward_velocity_mps, ag.forward) ==
             std::tie(bg.wheelbase_m, bg.maximum_steering_angle_rad,
                      bg.far_turn_radius_m, bg.arc_interval_m,
                      bg.after_straight_distance_m, bg.forward_velocity_mps,
                      bg.backward_velocity_mps, bg.forward) &&
         std::tie(ap.minimum_lateral_jerk_mps3,
                  ap.maximum_lateral_jerk_mps3, ap.jerk_samples,
                  ap.pull_over_velocity_mps,
                  ap.after_shift_straight_distance_m,
                  ap.interpolation_interval_m) ==
             std::tie(bp.minimum_lateral_jerk_mps3,
                      bp.maximum_lateral_jerk_mps3, bp.jerk_samples,
                      bp.pull_over_velocity_mps,
                      bp.after_shift_straight_distance_m,
                      bp.interpolation_interval_m) &&
         std::tie(av.front_m, av.back_m, av.half_width_m,
                  av.sample_spacing_m) ==
             std::tie(bv.front_m, bv.back_m, bv.half_width_m,
                      bv.sample_spacing_m) &&
         std::tie(af.wheelbase_m, af.maximum_steering_angle_rad,
                  af.expansion_distance_m, af.velocity_mps, af.front_m,
                  af.rear_m, af.half_width_m, af.vehicle_shape_margin_m,
                  af.goal_position_tolerance_m,
                  af.goal_yaw_tolerance_rad,
                  af.final_straight_distance_m,
                  af.use_backward_goal_approach, af.reverse_weight,
                  af.direction_change_weight, af.theta_bins,
                  af.obstacle_threshold, af.maximum_expansions,
                  af.use_reverse, af.use_reeds_shepp_heuristic,
                  af.use_obstacle_distance_heuristic, af.use_rrt_star,
                  af.rrt_iterations, af.rrt_step_m, af.rrt_time_limit_ms,
                  af.rrt_seed) ==
             std::tie(bf.wheelbase_m, bf.maximum_steering_angle_rad,
                      bf.expansion_distance_m, bf.velocity_mps, bf.front_m,
                      bf.rear_m, bf.half_width_m, bf.vehicle_shape_margin_m,
                      bf.goal_position_tolerance_m,
                      bf.goal_yaw_tolerance_rad,
                      bf.final_straight_distance_m,
                      bf.use_backward_goal_approach, bf.reverse_weight,
                      bf.direction_change_weight, bf.theta_bins,
                      bf.obstacle_threshold, bf.maximum_expansions,
                      bf.use_reverse, bf.use_reeds_shepp_heuristic,
                      bf.use_obstacle_distance_heuristic, bf.use_rrt_star,
                      bf.rrt_iterations, bf.rrt_step_m, bf.rrt_time_limit_ms,
                      bf.rrt_seed) && grid_matches();
}

std::size_t nearestIndex(const planning::Trajectory &path,
                         const planning::TrajectoryPoint &ego) {
  std::size_t nearest = 0;
  double distance = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0; index < path.size(); ++index) {
    const double candidate = std::hypot(path[index].x - ego.x,
                                        path[index].y - ego.y);
    if (candidate < distance) { distance = candidate; nearest = index; }
  }
  return nearest;
}

std::vector<double> motionArrivalTimes(
    const planning::Trajectory &path,
    const planning::TrajectoryPoint &ego) {
  std::vector<double> times(path.size(), 0.0);
  if (path.size() < 2) return times;
  std::size_t nearest_segment = 0;
  double best = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0; index + 1 < path.size(); ++index) {
    const double dx = path[index + 1].x - path[index].x;
    const double dy = path[index + 1].y - path[index].y;
    const double length_squared = dx * dx + dy * dy;
    const double ratio = length_squared > 1.0e-12 ? std::clamp(
        ((ego.x - path[index].x) * dx +
         (ego.y - path[index].y) * dy) / length_squared, 0.0, 1.0) : 0.0;
    const double distance = std::hypot(
        ego.x - path[index].x - ratio * dx,
        ego.y - path[index].y - ratio * dy);
    if (distance < best) { best = distance; nearest_segment = index; }
  }
  // motion_utils::calculate_time_from_start initializes every point to zero,
  // then integrates only after the segment nearest the current ego position.
  for (std::size_t index = nearest_segment + 1;
       index < path.size(); ++index) {
    const double distance = std::hypot(
        path[index].x - path[index - 1].x,
        path[index].y - path[index - 1].y);
    times[index] = times[index - 1] + distance / std::max(
        1.0, static_cast<double>(
            path[index - 1].longitudinal_velocity_mps));
  }
  return times;
}

std::optional<planning::MotionVelocityModule> motionPluginClass(
    std::string_view name) {
  using Module = planning::MotionVelocityModule;
  if (name == "autoware::motion_velocity_planner::OutOfLaneModule")
    return Module::OutOfLane;
  if (name ==
      "autoware::motion_velocity_planner::ObstacleVelocityLimiterModule")
    return Module::ObstacleVelocityLimiter;
  if (name ==
      "autoware::motion_velocity_planner::DynamicObstacleStopModule")
    return Module::DynamicObstacleStop;
  return std::nullopt;
}

double outOfLaneMinimumStopArc(
    const planning::Trajectory &path, const planning::TrajectoryPoint &ego,
    const planning::PlanningOutOfLaneBrakingInput &braking,
    const std::optional<planning::MotionVelocityPoint2D> previous) {
  if (!std::isfinite(braking.ego_velocity_mps) ||
      !std::isfinite(braking.ego_acceleration_mps2) ||
      !std::isfinite(braking.minimum_acceleration_mps2) ||
      !std::isfinite(braking.minimum_jerk_mps3) ||
      braking.minimum_acceleration_mps2 >= 0.0 ||
      braking.minimum_jerk_mps3 >= 0.0)
    throw std::invalid_argument("invalid out-of-lane braking callback input");
  // PlannerData::calculate_min_deceleration_distance(0.0), followed by
  // calculate_min_stop_and_slowdown_distances. The latter also considers the
  // previous inserted point before the decision timer is updated.
  const double distance = planning::calcCrosswalkDecelDistance(
      braking.ego_velocity_mps, 0.0, braking.ego_acceleration_mps2,
      braking.minimum_acceleration_mps2,
      std::abs(braking.minimum_jerk_mps3),
      braking.minimum_jerk_mps3).value_or(0.0);
  const planning::MotionVelocityPoint2D position{ego.x, ego.y};
  const double ego_arc = planning::dynamicStopArc(path, position);
  if (!previous) return std::max(0.0, ego_arc + distance);
  // The source compares the previous point's arc from the first retained
  // trajectory index with the braking distance from ego, then adds the ego
  // offset within that first segment.
  double first_segment_arc = 0.0;
  double best_distance = std::numeric_limits<double>::infinity();
  double prefix = 0.0;
  for (std::size_t index = 0; index + 1 < path.size(); ++index) {
    const double dx = path[index + 1].x - path[index].x;
    const double dy = path[index + 1].y - path[index].y;
    const double length_squared = dx * dx + dy * dy;
    const double ratio = length_squared > 0.0 ? std::clamp(
        ((ego.x - path[index].x) * dx +
         (ego.y - path[index].y) * dy) / length_squared,
        0.0, 1.0) : 0.0;
    const double lateral = std::hypot(
        ego.x - path[index].x - ratio * dx,
        ego.y - path[index].y - ratio * dy);
    if (lateral < best_distance) {
      best_distance = lateral;
      first_segment_arc = prefix;
    }
    prefix += std::sqrt(length_squared);
  }
  const double previous_arc = planning::dynamicStopArc(path, *previous);
  return std::max(0.0, ego_arc +
      std::min(previous_arc - first_segment_arc, distance));
}

// behavior_path_planner: centerline input and published path both use a 2 m
// distance grid. For an active goal module, resamplePathWithSpline first
// retains input points, then adds grid samples only where none lies within
// 0.2 m. Otherwise the output pass retains the first point of each lane ID.
planning::BehaviorGoalPath resampleRouteReference(
    const planning::BehaviorGoalPath &input, bool published_path,
    bool keep_input_points = false) {
  if (input.points.size() < 2) return input;
  std::vector<double> arcs(input.points.size(), 0.0);
  for (std::size_t i = 1; i < arcs.size(); ++i)
    arcs[i] = arcs[i - 1] + std::hypot(
        input.points[i].x - input.points[i - 1].x,
        input.points[i].y - input.points[i - 1].y);
  if (arcs.back() < 0.01) return input;
  std::vector<double> samples;
  if (!published_path) {
    for (double arc = 0.0; arc < arcs.back(); arc += 2.0)
      samples.push_back(arc);
    // motion_utils::resamplePath replaces the final grid sample when the
    // terminal arc is within overlap_threshold (0.1 m).
    if (arcs.back() - samples.back() < 0.1)
      samples.back() = arcs.back();
    else
      samples.push_back(arcs.back());
  }
  std::unordered_set<std::int64_t> seen_lanes;
  bool first_stop_retained = false;
  double first_stop_arc = -1.0;
  for (std::size_t i = 0; i < arcs.size(); ++i) {
    bool retain = keep_input_points || input.lane_ids[i].size() > 1;
    for (const auto id : input.lane_ids[i]) {
      if (published_path && seen_lanes.insert(id).second)
        retain = true;
    }
    if (retain && !published_path) {
      // motion_utils::resamplePath inserts multi-lane-ID input points by
      // replacing a grid sample within the 0.1 m overlap threshold.
      for (std::size_t j = 1; j < samples.size(); ++j) {
        if (samples[j - 1] > arcs[i] || arcs[i] >= samples[j]) continue;
        if (arcs[i] - samples[j - 1] < 0.1)
          samples[j - 1] = arcs[i];
        else if (samples[j] - arcs[i] < 0.1)
          samples[j] = arcs[i];
        else
          samples.insert(samples.begin() + j, arcs[i]);
        break;
      }
    } else if (retain && std::none_of(
                   samples.begin(), samples.end(), [&](double sample) {
                     return std::abs(sample - arcs[i]) < 0.2;
                   })) {
      samples.push_back(arcs[i]);
    }
    if (published_path && !first_stop_retained &&
        std::abs(input.points[i].longitudinal_velocity_mps) < 1.0e-3) {
      samples.push_back(arcs[i]);
      first_stop_retained = true;
      first_stop_arc = arcs[i];
    }
  }
  if (published_path) {
    for (double arc = 0.0; arc < arcs.back(); arc += 2.0)
      if (std::none_of(samples.begin(), samples.end(), [&](double sample) {
            return std::abs(sample - arc) < 0.2;
          })) samples.push_back(arc);
    if (std::none_of(samples.begin(), samples.end(), [&](double sample) {
          return std::abs(sample - arcs.back()) < 0.2;
        })) samples.push_back(arcs.back());
  }
  std::sort(samples.begin(), samples.end());
  samples.erase(std::unique(samples.begin(), samples.end(),
                            [published_path](double a, double b) {
                              return std::abs(a - b) <
                                  (published_path ? 0.2 : 0.01);
                            }), samples.end());
  if (first_stop_retained) {
    const auto nearest = std::min_element(
        samples.begin(), samples.end(), [first_stop_arc](double a, double b) {
          return std::abs(a - first_stop_arc) <
                 std::abs(b - first_stop_arc);
        });
    if (nearest != samples.end() &&
        std::abs(*nearest - first_stop_arc) < 0.2) {
      *nearest = first_stop_arc;
      std::sort(samples.begin(), samples.end());
      bool kept_stop = false;
      samples.erase(std::remove_if(samples.begin(), samples.end(),
                    [&](double arc) {
                      if (std::abs(arc - first_stop_arc) >= 0.2)
                        return false;
                      if (arc == first_stop_arc && !kept_stop) {
                        kept_stop = true;
                        return false;
                      }
                      return true;
                    }), samples.end());
    }
  }
  if (samples.size() < 2) return input;
  planning::BehaviorGoalPath output = input;
  output.points = planning::resampleJerkAtArcs(input.points, samples);
  // motion_utils::resamplePoseVector derives heading from the sampled
  // positions; interpolating the source quaternions changes intersection
  // footprint and stop-line decisions on curved route sections.
  for (std::size_t i = 0; i + 1 < output.points.size(); ++i) {
    auto &point = output.points[i];
    const auto &next = output.points[i + 1];
    const double dx = next.x - point.x;
    const double dy = next.y - point.y;
    const double yaw = std::atan2(dy, dx);
    const double pitch = std::atan2(next.z - point.z, std::hypot(dx, dy));
    const double sp = std::sin(pitch / 2.0);
    const double cp = std::cos(pitch / 2.0);
    const double sy = std::sin(yaw / 2.0);
    const double cy = std::cos(yaw / 2.0);
    point.orientation_x = -sp * sy;
    point.orientation_y = sp * cy;
    point.orientation_z = cp * sy;
    point.orientation_w = cp * cy;
  }
  output.points.back().orientation_x =
      output.points[output.points.size() - 2].orientation_x;
  output.points.back().orientation_y =
      output.points[output.points.size() - 2].orientation_y;
  output.points.back().orientation_z =
      output.points[output.points.size() - 2].orientation_z;
  output.points.back().orientation_w =
      output.points[output.points.size() - 2].orientation_w;
  output.lane_ids.clear();
  output.lane_ids.reserve(samples.size());
  for (const double arc : samples) {
    const auto upper = std::upper_bound(arcs.begin(), arcs.end(), arc);
    const std::size_t segment = std::min<std::size_t>(
        static_cast<std::size_t>(upper - arcs.begin() - 1), arcs.size() - 2);
    const auto &before = input.lane_ids[segment];
    const auto &after = input.lane_ids[segment + 1];
    std::vector<std::int64_t> ids;
    if (std::abs(arc - arcs[segment + 1]) < 1.0e-6) {
      ids = after;
    } else if (std::abs(arc - arcs[segment]) < 1.0e-6) {
      ids = before;
    } else {
      for (const auto id : before)
        if (std::find(after.begin(), after.end(), id) != after.end())
          ids.push_back(id);
      if (ids.empty()) ids = before;
    }
    output.lane_ids.push_back(std::move(ids));
  }
  return output;
}

// getReferencePath limits the lane sequence to the goal's centerline arc
// before goal-pose refinement. Keep the projected terminal point explicitly.
planning::BehaviorGoalPath truncateRouteAtGoal(
    const planning::BehaviorGoalPath &input,
    const planning::BehaviorGoalPose &goal, std::int64_t goal_lane_id) {
  if (input.points.size() < 2) return input;
  std::size_t selected = input.points.size();
  double selected_ratio = 0.0;
  double best = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i + 1 < input.points.size(); ++i) {
    const auto contains_goal_lane = [&](std::size_t index) {
      const auto &ids = input.lane_ids[index];
      return std::find(ids.begin(), ids.end(), goal_lane_id) != ids.end();
    };
    if (!contains_goal_lane(i) && !contains_goal_lane(i + 1)) continue;
    const auto &a = input.points[i];
    const auto &b = input.points[i + 1];
    const double dx = b.x - a.x, dy = b.y - a.y;
    const double length2 = dx * dx + dy * dy;
    if (length2 < 1.0e-9) continue;
    const double ratio = std::clamp(
        ((goal.x - a.x) * dx + (goal.y - a.y) * dy) / length2,
        0.0, 1.0);
    const double error = std::hypot(goal.x - a.x - ratio * dx,
                                    goal.y - a.y - ratio * dy);
    if (error < best) {
      best = error;
      selected = i;
      selected_ratio = ratio;
    }
  }
  if (selected >= input.points.size() - 1) return input;
  planning::BehaviorGoalPath result = input;
  const auto &a = input.points[selected];
  const auto &b = input.points[selected + 1];
  auto terminal = a;
  terminal.x = a.x + selected_ratio * (b.x - a.x);
  terminal.y = a.y + selected_ratio * (b.y - a.y);
  terminal.z = a.z + selected_ratio * (b.z - a.z);
  terminal.orientation_z = std::sin(std::atan2(b.y - a.y, b.x - a.x) / 2.0);
  terminal.orientation_w = std::cos(std::atan2(b.y - a.y, b.x - a.x) / 2.0);
  result.points.resize(selected + 1);
  result.lane_ids.resize(selected + 1);
  if (selected_ratio > 1.0e-6) {
    result.points.push_back(terminal);
    result.lane_ids.push_back({goal_lane_id});
  }
  return result;
}

// Follow motion_utils::cropPoints' forward-then-backward order. Retain the
// point crossing the backward threshold so sparse source centerlines still
// cover the requested 5 m plus the 2 m input sampling margin.
planning::BehaviorGoalPath cropRouteReference(
    const planning::BehaviorGoalPath &input,
    const planning::TrajectoryPoint &ego,
    double backward_path_length_m,
    bool source_crop_index = false) {
  if (input.points.size() < 2) return input;
  std::size_t segment = 0;
  double offset = 0.0;
  double best = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i + 1 < input.points.size(); ++i) {
    const auto &a = input.points[i];
    const auto &b = input.points[i + 1];
    const double dx = b.x - a.x, dy = b.y - a.y;
    const double length2 = dx * dx + dy * dy;
    if (length2 < 1.0e-9) continue;
    const double ratio = std::clamp(
        ((ego.x - a.x) * dx + (ego.y - a.y) * dy) / length2,
        0.0, 1.0);
    const double distance = std::hypot(
        ego.x - a.x - ratio * dx, ego.y - a.y - ratio * dy);
    if (distance < best) {
      best = distance;
      segment = i;
      offset = ratio * std::sqrt(length2);
    }
  }
  if (source_crop_index) {
    // motion_utils::findFirstNearestSegmentIndexWithSoftConstraints first
    // selects a path point in the first contiguous valid distance/yaw run.
    // Its distance-only and unconstrained searches are fallback passes.
    const double ego_yaw = std::atan2(
        2.0 * (ego.orientation_w * ego.orientation_z +
               ego.orientation_x * ego.orientation_y),
        1.0 - 2.0 * (ego.orientation_y * ego.orientation_y +
                     ego.orientation_z * ego.orientation_z));
    const auto nearest_point = [&](bool check_distance, bool check_yaw) {
      double nearest_squared = std::numeric_limits<double>::infinity();
      std::optional<std::size_t> selected;
      for (std::size_t i = 0; i < input.points.size(); ++i) {
        const auto &point = input.points[i];
        const double dx = point.x - ego.x;
        const double dy = point.y - ego.y;
        const double squared = dx * dx + dy * dy;
        const double point_yaw = std::atan2(
            2.0 * (point.orientation_w * point.orientation_z +
                   point.orientation_x * point.orientation_y),
            1.0 - 2.0 * (point.orientation_y * point.orientation_y +
                         point.orientation_z * point.orientation_z));
        const bool valid = (!check_distance || squared <= 9.0) &&
            (!check_yaw ||
             std::abs(std::remainder(point_yaw - ego_yaw,
                                     2.0 * std::acos(-1.0))) <= 1.046);
        if (!valid) {
          if (selected && check_distance) break;
          continue;
        }
        if (squared < nearest_squared) {
          nearest_squared = squared;
          selected = i;
        }
      }
      return selected;
    };
    auto selected_point = nearest_point(true, true);
    if (!selected_point) selected_point = nearest_point(true, false);
    if (!selected_point) selected_point = nearest_point(false, false);
    const auto selected = *selected_point;
    if (selected == 0) segment = 0;
    else if (selected + 1 == input.points.size())
      segment = input.points.size() - 2;
    else {
      const auto &a = input.points[selected];
      const auto &b = input.points[selected + 1];
      const double segment_length = std::hypot(b.x - a.x, b.y - a.y);
      const double signed_offset = segment_length < 1.0e-9 ? 0.0 :
          ((ego.x - a.x) * (b.x - a.x) +
           (ego.y - a.y) * (b.y - a.y)) /
          segment_length;
      segment = signed_offset <= 0.0 ? selected - 1 : selected;
    }
    const auto &a = input.points[segment];
    const auto &b = input.points[segment + 1];
    const double segment_length = std::hypot(b.x - a.x, b.y - a.y);
    offset = segment_length < 1.0e-9 ? 0.0 :
        ((ego.x - a.x) * (b.x - a.x) +
         (ego.y - a.y) * (b.y - a.y)) / segment_length;
  }
  std::size_t end = input.points.size();
  double distance = -offset;
  for (std::size_t i = segment + 1; i < input.points.size(); ++i) {
    distance += std::hypot(input.points[i].x - input.points[i - 1].x,
                           input.points[i].y - input.points[i - 1].y);
    if (distance > 300.0) { end = i; break; }
  }
  if (end < 2) return input;
  segment = std::min(segment, end - 2);
  std::size_t begin = 0;
  distance = -offset;
  for (std::size_t i = segment; i > 0; --i) {
    distance -= std::hypot(input.points[i].x - input.points[i - 1].x,
                           input.points[i].y - input.points[i - 1].y);
    if (distance < -(backward_path_length_m + 2.0)) {
      // motion_utils::cropBackwardPoints returns points.begin() + i.
      // This index alone is not source-equivalent until the raw path grid and
      // callback ego pose are aligned; preserve the baseline separately.
      begin = source_crop_index ? i : i - 1;
      break;
    }
  }
  if (end - begin < 2) return input;
  planning::BehaviorGoalPath output = input;
  output.points.assign(input.points.begin() + begin,
                       input.points.begin() + end);
  output.lane_ids.assign(input.lane_ids.begin() + begin,
                         input.lane_ids.begin() + end);
  return output;
}

// RouteHandler builds a lanelet sequence around the current lane, clips its
// raw centerline at the ego arc, and only then resamples it. Keep the route
// lanelet boundaries here so the 2 m grid starts at the same arc as the
// source planner when the ego moves through a long lanelet.
struct RouteLaneRange {
  std::size_t first = 0;
  std::size_t last = 0;
};

planning::TrajectoryPoint projectEgoToRouteCenterline(
    const planning::BehaviorGoalPath &raw, const RouteLaneRange &lane,
    const planning::TrajectoryPoint &ego) {
  auto projected = ego;
  double best = std::numeric_limits<double>::infinity();
  for (std::size_t i = lane.first; i < lane.last; ++i) {
    const auto &a = raw.points[i];
    const auto &b = raw.points[i + 1];
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double length2 = dx * dx + dy * dy;
    if (length2 < 1.0e-9) continue;
    const double ratio =
        ((ego.x - a.x) * dx + (ego.y - a.y) * dy) / length2;
    const double nearest_ratio = std::clamp(ratio, 0.0, 1.0);
    const double x = a.x + nearest_ratio * dx;
    const double y = a.y + nearest_ratio * dy;
    const double distance2 = (ego.x - x) * (ego.x - x) +
                             (ego.y - y) * (ego.y - y);
    if (distance2 >= best) continue;
    best = distance2;
    // getClosestCenterPose selects the nearest finite segment, then projects
    // to its supporting line without clipping the projection to endpoints.
    projected.x = a.x + ratio * dx;
    projected.y = a.y + ratio * dy;
    projected.z = ego.z;
    const double yaw = std::atan2(dy, dx);
    projected.orientation_x = 0.0;
    projected.orientation_y = 0.0;
    projected.orientation_z = std::sin(yaw / 2.0);
    projected.orientation_w = std::cos(yaw / 2.0);
  }
  return projected;
}

planning::BehaviorGoalPath clipRawRouteReference(
    const planning::BehaviorGoalPath &raw,
    const std::vector<RouteLaneRange> &lanes,
    const planning::TrajectoryPoint &ego,
    double backward_path_length_m,
    const std::optional<planning::BehaviorGoalPose> &goal,
    std::int64_t goal_lane_id) {
  if (raw.points.size() < 2 || lanes.empty()) return raw;
  std::vector<double> arcs(raw.points.size(), 0.0);
  for (std::size_t i = 1; i < arcs.size(); ++i)
    arcs[i] = arcs[i - 1] + std::hypot(
        raw.points[i].x - raw.points[i - 1].x,
        raw.points[i].y - raw.points[i - 1].y);

  std::size_t current = 0;
  double ego_arc = 0.0;
  double nearest_distance = std::numeric_limits<double>::infinity();
  for (std::size_t lane = 0; lane < lanes.size(); ++lane) {
    const auto &range = lanes[lane];
    for (std::size_t i = range.first; i < range.last; ++i) {
      const auto &a = raw.points[i];
      const auto &b = raw.points[i + 1];
      const double dx = b.x - a.x;
      const double dy = b.y - a.y;
      const double length2 = dx * dx + dy * dy;
      if (length2 < 1.0e-9) continue;
      const double ratio = std::clamp(
          ((ego.x - a.x) * dx + (ego.y - a.y) * dy) / length2,
          0.0, 1.0);
      const double distance = std::hypot(
          ego.x - a.x - ratio * dx, ego.y - a.y - ratio * dy);
      if (distance < nearest_distance) {
        nearest_distance = distance;
        current = lane;
        ego_arc = arcs[i] + ratio * std::sqrt(length2);
      }
    }
  }
  if (!std::isfinite(nearest_distance)) return raw;

  const double backward_length = backward_path_length_m + 10.0;
  constexpr double forward_length = 300.0;
  std::size_t first_lane = current;
  if (ego_arc - arcs[lanes[current].first] < backward_length) {
    double preceding_length = 0.0;
    while (first_lane > 0 && preceding_length < backward_length) {
      --first_lane;
      preceding_length += arcs[lanes[first_lane].last] -
                          arcs[lanes[first_lane].first];
    }
  }
  std::size_t last_lane = current;
  double following_length = 0.0;
  while (last_lane + 1 < lanes.size() &&
         following_length < forward_length) {
    ++last_lane;
    following_length += arcs[lanes[last_lane].last] -
                        arcs[lanes[last_lane].first];
  }

  const double start = std::max(arcs[lanes[first_lane].first],
                                ego_arc - backward_length);
  double end = std::min(arcs[lanes[last_lane].last],
                        ego_arc + forward_length);
  if (goal && raw.lane_ids.size() == raw.points.size()) {
    double nearest_goal_distance = std::numeric_limits<double>::infinity();
    double goal_arc = end;
    for (std::size_t i = 0; i + 1 < raw.points.size(); ++i) {
      const auto contains_goal_lane = [&](std::size_t index) {
        const auto &ids = raw.lane_ids[index];
        return std::find(ids.begin(), ids.end(), goal_lane_id) != ids.end();
      };
      if (!contains_goal_lane(i) && !contains_goal_lane(i + 1)) continue;
      const auto &a = raw.points[i];
      const auto &b = raw.points[i + 1];
      const double dx = b.x - a.x, dy = b.y - a.y;
      const double length2 = dx * dx + dy * dy;
      if (length2 < 1.0e-9) continue;
      const double ratio = std::clamp(
          ((goal->x - a.x) * dx + (goal->y - a.y) * dy) / length2,
          0.0, 1.0);
      const double distance = std::hypot(
          goal->x - a.x - ratio * dx,
          goal->y - a.y - ratio * dy);
      if (distance < nearest_goal_distance) {
        nearest_goal_distance = distance;
        goal_arc = arcs[i] + ratio * std::sqrt(length2);
      }
    }
    if (std::isfinite(nearest_goal_distance)) end = std::min(end, goal_arc);
  }
  if (end - start < 0.01) return raw;
  if (start < 1.0e-9 && end >= arcs.back() - 1.0e-9) return raw;

  planning::BehaviorGoalPath clipped = raw;
  clipped.points.clear();
  clipped.lane_ids.clear();
  const auto append_at_arc = [&](double arc) {
    const auto upper = std::upper_bound(arcs.begin(), arcs.end(), arc);
    const std::size_t i = std::min<std::size_t>(
        std::max<std::size_t>(1, upper - arcs.begin()) - 1,
        arcs.size() - 2);
    const double length = arcs[i + 1] - arcs[i];
    const double ratio = length > 1.0e-9 ?
        std::clamp((arc - arcs[i]) / length, 0.0, 1.0) : 0.0;
    auto point = raw.points[i];
    point.x += ratio * (raw.points[i + 1].x - point.x);
    point.y += ratio * (raw.points[i + 1].y - point.y);
    point.z += ratio * (raw.points[i + 1].z - point.z);
    clipped.points.push_back(point);
    clipped.lane_ids.push_back(
        ratio >= 1.0 - 1.0e-9 ? raw.lane_ids[i + 1] : raw.lane_ids[i]);
  };
  append_at_arc(start);
  for (std::size_t i = 1; i + 1 < arcs.size(); ++i) {
    if (arcs[i] > start + 1.0e-9 && arcs[i] < end - 1.0e-9) {
      clipped.points.push_back(raw.points[i]);
      clipped.lane_ids.push_back(raw.lane_ids[i]);
    }
  }
  append_at_arc(end);
  return clipped;
}

void expandSideShiftBoundary(
    std::vector<planning::PathOptimizerPoint2D> &boundary,
    double offset_m) {
  if (boundary.size() < 2 || std::abs(offset_m) <= 1.0e-9) return;
  const auto original = boundary;
  for (std::size_t index = 0; index < boundary.size(); ++index) {
    const auto &before = original[index == 0 ? 0 : index - 1];
    const auto &after = original[
        std::min(index + 1, original.size() - 1)];
    const double dx = after.x - before.x;
    const double dy = after.y - before.y;
    const double length = std::hypot(dx, dy);
    if (length < 1.0e-9) continue;
    boundary[index].x -= offset_m * dy / length;
    boundary[index].y += offset_m * dx / length;
  }
}

double lateralOffset(const planning::Trajectory &path,
                     const planning::TrajectoryPoint &ego) {
  double nearest = std::numeric_limits<double>::infinity();
  double offset = 0.0;
  for (std::size_t index = 0; index + 1 < path.size(); ++index) {
    const auto &a = path[index];
    const auto &b = path[index + 1];
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double length_squared = dx * dx + dy * dy;
    if (length_squared < 1.0e-12) continue;
    const double ratio = std::clamp(
        ((ego.x - a.x) * dx + (ego.y - a.y) * dy) / length_squared,
        0.0, 1.0);
    const double residual_x = ego.x - a.x - ratio * dx;
    const double residual_y = ego.y - a.y - ratio * dy;
    const double distance = std::hypot(residual_x, residual_y);
    if (distance < nearest) {
      nearest = distance;
      offset = (dx * residual_y - dy * residual_x) /
               std::sqrt(length_squared);
    }
  }
  return offset;
}

double longitudinalArc(const planning::Trajectory &path,
                       double x, double y) {
  double nearest = std::numeric_limits<double>::infinity();
  double selected_arc = 0.0;
  double traveled = 0.0;
  for (std::size_t index = 0; index + 1 < path.size(); ++index) {
    const double dx = path[index + 1].x - path[index].x;
    const double dy = path[index + 1].y - path[index].y;
    const double length = std::hypot(dx, dy);
    if (length < 1.0e-9) continue;
    const double ratio = std::clamp(
        ((x - path[index].x) * dx +
         (y - path[index].y) * dy) / (length * length),
        0.0, 1.0);
    const double error = std::hypot(
        x - path[index].x - ratio * dx,
        y - path[index].y - ratio * dy);
    if (error < nearest) {
      nearest = error;
      selected_arc = traveled + ratio * length;
    }
    traveled += length;
  }
  return selected_arc;
}

double pointYaw(const planning::TrajectoryPoint &point) {
  return std::atan2(
      2.0 * (point.orientation_w * point.orientation_z +
             point.orientation_x * point.orientation_y),
      1.0 - 2.0 * (point.orientation_y * point.orientation_y +
                   point.orientation_z * point.orientation_z));
}

planning::BehaviorSafetyPolygon safetyPolygon(
    const std::vector<planning::BehaviorPathPoint2d> &points) {
  planning::BehaviorSafetyPolygon polygon;
  for (const auto &point : points)
    polygon.emplace_back(point.x, point.y);
  return polygon;
}

planning::BehaviorSafetyPolygon expandedLanePolygon(
    const planning::LaneletBinAdapter &map, std::int64_t lane_id,
    double left_offset_m, double right_offset_m) {
  const auto bounds = map.laneBounds(lane_id);
  if (bounds.left.size() < 2 || bounds.right.size() < 2) return {};
  lanelet::BasicLineString2d left;
  lanelet::BasicLineString2d right;
  for (const auto &point : bounds.left)
    left.emplace_back(point.x, point.y);
  for (const auto &point : bounds.right)
    right.emplace_back(point.x, point.y);
  // Match autoware_lanelet2_extension::getExpandedLanelet: offsetNoThrow
  // handles corners of curved bounds, while per-vertex normal displacement
  // can fold a lane boundary back on itself.
  auto expanded_left = lanelet::geometry::offsetNoThrow(left, left_offset_m);
  auto expanded_right = lanelet::geometry::offsetNoThrow(right, -right_offset_m);
  if (expanded_left.size() < 2 || expanded_right.size() < 2) return {};
  const auto cap = [&](std::size_t left_index, std::size_t right_index,
                       std::size_t expanded_left_index,
                       std::size_t expanded_right_index) {
    const double dx = right[right_index].x() - left[left_index].x();
    const double dy = right[right_index].y() - left[left_index].y();
    const double norm = std::hypot(dx, dy);
    if (norm < 1.0e-9) return;
    expanded_left[expanded_left_index].x() =
        left[left_index].x() - left_offset_m * dx / norm;
    expanded_left[expanded_left_index].y() =
        left[left_index].y() - left_offset_m * dy / norm;
    expanded_right[expanded_right_index].x() =
        right[right_index].x() + right_offset_m * dx / norm;
    expanded_right[expanded_right_index].y() =
        right[right_index].y() + right_offset_m * dy / norm;
  };
  cap(0, 0, 0, 0);
  cap(left.size() - 1, right.size() - 1,
      expanded_left.size() - 1, expanded_right.size() - 1);
  planning::BehaviorSafetyPolygon polygon;
  for (const auto &point : expanded_left)
    polygon.emplace_back(point.x(), point.y());
  for (std::size_t index = expanded_right.size(); index > 0; --index)
    polygon.emplace_back(expanded_right[index - 1].x(),
                         expanded_right[index - 1].y());
  return polygon;
}

std::vector<planning::BehaviorLaneChangeSelectableObject>
laneChangePerceptionObjects(
    const planning::Trajectory &reference,
    const planning::TrajectoryPoint &ego,
    const planning::PlanningCruisePerceptionFrame &perception,
    double callback_time_s,
    const std::vector<std::vector<planning::BehaviorPathPoint2d>> &current,
    const std::vector<std::vector<planning::BehaviorPathPoint2d>> &target,
    const planning::BehaviorSafetyPolygon &expanded_target) {
  std::vector<planning::BehaviorLaneChangeSelectableObject> selected;
  if (reference.size() < 2 || current.empty() || target.empty())
    return selected;
  const auto current_polygon = safetyPolygon(current.front());
  const auto target_polygon = safetyPolygon(target.front());
  const double ego_arc = longitudinalArc(reference, ego.x, ego.y);
  double terminal_arc = 0.0;
  for (std::size_t index = 1; index < reference.size(); ++index)
    terminal_arc += std::hypot(
        reference[index].x - reference[index - 1].x,
        reference[index].y - reference[index - 1].y);
  const double elapsed_s = perception.stamp_s > 0.0 ?
      std::max(0.0, callback_time_s - perception.stamp_s) : 0.0;
  for (const auto &object : perception.objects) {
    // toPolygon2d uses the footprint for POLYGON and diameter x for
    // CYLINDER; their unused dimensions may be zero in perception messages.
    const bool valid_shape =
        (object.shape_type == 0 && object.length_m > 0.0 &&
         object.width_m > 0.0) ||
        (object.shape_type == 1 && object.length_m > 0.0) ||
        (object.shape_type == 2 && object.footprint.size() >= 3);
    if (!valid_shape || !std::isfinite(object.x) ||
        !std::isfinite(object.y)) continue;
    planning::BehaviorLaneChangeSelectableObject entry;
    auto &target_object = entry.target;
    target_object.object.shape_type = object.shape_type;
    target_object.object.length_m = object.length_m;
    target_object.object.width_m = object.width_m;
    for (const auto &point : object.footprint)
      target_object.object.footprint.emplace_back(point.x, point.y);
    const double speed = std::hypot(
        object.velocity_x_mps, object.velocity_y_mps);
    const auto current = planning::currentCruiseObjectPose(
        {object.x, object.y, object.yaw_rad},
        object.predicted_paths, elapsed_s);
    const planning::BehaviorSafetyPose initial{
        0.0, current.x, current.y, current.yaw_rad, speed};
    if (!object.predicted_paths.empty()) {
      for (const auto &source : object.predicted_paths) {
        if (source.time_step_s <= 0.0) continue;
        planning::BehaviorSafetyPredictedPath path;
        path.confidence = source.confidence;
        if (elapsed_s > 0.0) path.poses.push_back(initial);
        for (std::size_t index = 0; index < source.poses.size(); ++index) {
          const auto &pose = source.poses[index];
          const double prediction_time =
              index * source.time_step_s - elapsed_s;
          if (prediction_time < 0.0) continue;
          double predicted_speed = speed;
          if (source.poses.size() >= 2) {
            const auto previous = index == 0 ? 0 : index - 1;
            const auto next = index == 0 ? 1 : index;
            predicted_speed = std::hypot(
                source.poses[next].x - source.poses[previous].x,
                source.poses[next].y - source.poses[previous].y) /
                source.time_step_s;
          }
          path.poses.push_back({
              prediction_time, pose.x, pose.y,
              pose.yaw_rad, predicted_speed});
        }
        if (!path.poses.empty())
          target_object.predicted_paths.push_back(std::move(path));
      }
    }
    if (target_object.predicted_paths.empty()) {
      planning::BehaviorSafetyPredictedPath path;
      for (int second = 0; second <= 10; ++second) {
        path.poses.push_back({
            static_cast<double>(second),
            current.x + second * object.velocity_x_mps,
            current.y + second * object.velocity_y_mps,
            current.yaw_rad, speed});
      }
      target_object.predicted_paths.push_back(std::move(path));
    }
    target_object.initial_velocity_mps = speed;
    auto &filter = entry.filter;
    filter.initial_polygon = planning::behaviorSafetyWorldPolygon(
        initial, target_object.object);
    filter.predicted_paths = target_object.predicted_paths;
    filter.current_lane = current_polygon;
    filter.target_lane = target_polygon;
    filter.expanded_target_lane = expanded_target.size() >= 3 ?
        expanded_target : target_polygon;
    planning::TrajectoryPoint object_point;
    object_point.x = current.x;
    object_point.y = current.y;
    filter.object_lateral_from_current_center_m =
        lateralOffset(reference, object_point);
    filter.ego_lateral_from_current_center_m =
        lateralOffset(reference, ego);
    filter.vehicle_width_m = 1.896;
    filter.initial_velocity_mps = speed;
    filter.ego_yaw_rad = pointYaw(ego);
    filter.object_yaw_rad = current.yaw_rad;
    filter.is_vehicle = object.classification >= 1 &&
                        object.classification <= 5;
    const double object_arc = longitudinalArc(
        reference, current.x, current.y);
    filter.ahead_of_ego = object_arc >= ego_arc;
    filter.before_terminal = object_arc + 1.0e-3 < terminal_arc;
    filter.type_allowed = object.classification >= 1 &&
                          object.classification <= 7;
    entry.signed_distance_from_ego_m = object_arc - ego_arc;
    selected.push_back(std::move(entry));
  }
  return selected;
}
}  // 익명 네임스페이스

namespace planning {

PlanningPipelineMapAdapter::PlanningPipelineMapAdapter(
    std::vector<std::uint8_t> map_data,
    std::vector<BehaviorVelocityModule> modules,
    PathOptimizerCoreParameters optimizer_parameters,
    PrecheckThresholds validator_thresholds,
    InvalidTrajectoryPolicy invalid_policy,
    ObstacleCruiseStopParameters cruise_parameters,
    CruiseCandidateParameters cruise_candidate_parameters,
    CruiseVehicleGeometry cruise_vehicle,
    bool experimental_route_sequence_clip,
    double route_backward_path_length_m,
    VelocityLimitSelectorConfig selector_config,
    CruisePidParameters cruise_pid_parameters)
    : map_data_(std::move(map_data)),
      modules_(std::move(modules)),
      map_planner_(std::make_unique<BehaviorVelocityMapPlanner>(
          map_data_, modules_)),
      route_map_(map_data_),
      experimental_route_sequence_clip_(experimental_route_sequence_clip),
      route_backward_path_length_m_(route_backward_path_length_m),
      cruise_vehicle_(cruise_vehicle),
      cruise_candidates_(cruise_candidate_parameters, cruise_vehicle_),
      cruise_pid_(cruise_pid_parameters),
      selector_config_(selector_config),
      velocity_limit_selector_(selector_config_),
      cruise_slowdown_([&] {
        ObstacleCruiseSlowdownParameters parameters;
        parameters.ego_front_offset = cruise_vehicle_.front_m;
        return parameters;
      }()),
      pipeline_(
          [this](const BehaviorGoalPath &path,
                 const TrajectoryPoint &ego, double now_s) {
            auto input = velocity_context_;
            input.trajectory = path.points;
            input.lane_ids = path.lane_ids;
            input.ego_x = ego.x;
            input.ego_y = ego.y;
            input.ego_velocity_mps = ego.longitudinal_velocity_mps;
            input.now_s = now_s;
            last_velocity_result_ = map_planner_->apply(input);
            return last_velocity_result_->trajectory;
          },
          optimizer_parameters, validator_thresholds,
          invalid_policy, cruise_parameters,
          [this](const Trajectory &path, const TrajectoryPoint &ego,
                 double now_s) {
            std::vector<MotionVelocityPlan> plans;
            for (const auto module : motion_modules_) {
              if (module == MotionVelocityModule::OutOfLane) {
                if (out_of_lane_input_)
                  plans.push_back(planOutOfLane(
                      path, ego, now_s, *out_of_lane_input_));
              } else if (module == MotionVelocityModule::ObstacleVelocityLimiter) {
                if (pointcloud_.velocity_limiter_enabled &&
                    !pointcloud_.velocity_limiter_points.empty()) {
                  auto times = pointcloud_.velocity_limiter_time_from_start_s;
                  if (times.size() != path.size()) {
                    times.assign(path.size(), 0.0);
                    for (std::size_t index = 1; index < path.size(); ++index) {
                      const double length = std::hypot(
                          path[index].x - path[index - 1].x,
                          path[index].y - path[index - 1].y);
                      times[index] = times[index - 1] + length / std::max(
                          0.1, static_cast<double>(
                              path[index - 1].longitudinal_velocity_mps));
                    }
                  }
                  plans.push_back(planObstacleVelocityLimitPointCloud(
                      path, times, nearestIndex(path, ego),
                      ego.longitudinal_velocity_mps,
                      pointcloud_.velocity_limiter_points,
                      pointcloud_.velocity_limiter_masks,
                      pointcloud_.velocity_limiter_parameters, {}, true));
                }
              } else if (module == MotionVelocityModule::DynamicObstacleStop) {
                plans.push_back(dynamic_stop_.plan(
                    path, ego, velocity_context_.dynamic_stop_objects,
                    0.0, 0.0, now_s));
              }
            }
            return plans;
          },
          [this](const Trajectory &path, const TrajectoryPoint &ego,
                 double ego_speed_mps, double now_s) {
            pointcloud_obstacles_.clear();
            if ((pointcloud_.cruise_stop_enabled ||
                 pointcloud_.cruise_slowdown_enabled) &&
                !pointcloud_.cruise_points.empty()) {
              pointcloud_obstacles_ = convertCruisePointcloudToObstacles(
                  pointcloud_.cruise_points, path, nearestIndex(path, ego),
                  pointcloud_.cruise_parameters);
            }
            auto candidates = cruise_candidates_.plan(
                path, ego, ego_speed_mps, now_s, cruise_perception_);
            if (pointcloud_.cruise_stop_enabled) {
              for (const auto &obstacle : pointcloud_obstacles_) {
                if (!obstacle.stop_collision_point) continue;
                ObstacleCruiseStopCandidate candidate;
                candidate.classification = 1;
                candidate.collision_distance_from_start = dynamicStopArc(
                    path, {obstacle.stop_collision_point->x,
                           obstacle.stop_collision_point->y});
                candidate.desired_margin = 3.0;
                candidates.push_back(candidate);
              }
            }
            return candidates;
          },
          [this](const Trajectory &source, const Trajectory &path,
                 const TrajectoryPoint &ego,
                 double ego_speed_mps, double now_s) {
            if (!pointcloud_.cruise_slowdown_enabled &&
                !cruise_perception_.slowdown_enabled) {
              last_slowdown_candidate_count_ = 0;
              last_slowdown_intervals_.clear();
              cruise_slowdown_candidates_.reset();
              return path;
            }
            cruise_slowdown_candidates_.beginFrame();
            std::vector<ObstacleCruiseSlowdownCandidate> slowdowns;
            for (std::size_t index = 0;
                 index < pointcloud_obstacles_.size(); ++index) {
              const auto &obstacle = pointcloud_obstacles_[index];
              if (obstacle.stop_collision_point) continue;
              if (!obstacle.slowdown_front_collision_point) continue;
              const auto &front = *obstacle.slowdown_front_collision_point;
              const auto &back = obstacle.slowdown_back_collision_point ?
                  *obstacle.slowdown_back_collision_point : front;
              ObstacleCruiseSlowdownCandidate candidate;
              candidate.id = std::to_string(index);
              candidate.front_collision_arc = dynamicStopArc(
                  source, {front.x, front.y});
              candidate.back_collision_arc = dynamicStopArc(
                  source, {back.x, back.y});
              candidate.lateral_distance =
                  obstacle.signed_lateral_distance_m;
              slowdowns.push_back(candidate);
            }
            if (cruise_perception_.slowdown_enabled) {
              const auto ego_index = nearestIndex(source, ego);
              const auto sampled = resampleCruiseCollisionTrajectory(
                  source, std::min(ego_index, source.size() - 2), 2.0, 6.0);
              if (sampled.size() < 2) {
                last_slowdown_candidate_count_ = slowdowns.size();
                cruise_slowdown_candidates_.endFrame();
                auto slowdown_result = cruise_slowdown_.plan(
                    path, {ego.x, ego.y}, ego_speed_mps,
                    cruise_acceleration_mps2_, slowdowns);
                last_slowdown_intervals_ =
                    slowdown_result.intervals.slowdowns;
                return slowdown_result.trajectory;
              }
              for (const auto &object : cruise_perception_.objects) {
                const auto &claimed =
                    cruise_candidates_.claimedObjectIds();
                if (std::find(claimed.begin(), claimed.end(),
                              object.id) != claimed.end())
                  continue;
                if (object.id.empty() || object.length_m <= 0.0 ||
                    object.width_m <= 0.0 || object.classification == 0 ||
                    object.classification > 7)
                  continue;
                const double elapsed = cruise_perception_.stamp_s > 0.0 ?
                    std::max(0.0, now_s -
                                   cruise_perception_.stamp_s) : 0.0;
                const auto pose = currentCruiseObjectPose(
                    {object.x, object.y, object.yaw_rad},
                    object.predicted_paths, elapsed);
                const auto projection = projectCruiseObjectToTrajectory(
                    source, ego_index,
                    {pose.x, pose.y, pose.yaw_rad,
                     object.velocity_x_mps, object.velocity_y_mps});
                if (!projection || !passesObstacleCruiseRoughLateralFilter(
                        *projection,
                        std::max(object.length_m, object.width_m),
                        cruise_vehicle_.width_m))
                  continue;
                const CruiseObjectShape shape{
                    object.shape_type == 2 && object.footprint.empty() ?
                        0 : object.shape_type,
                    object.length_m, object.width_m, object.footprint};
                const CruisePredictedPose ego_pose{ego.x, ego.y,
                    std::atan2(2.0 * (ego.orientation_w * ego.orientation_z +
                                      ego.orientation_x * ego.orientation_y),
                               1.0 - 2.0 *
                                   (ego.orientation_y * ego.orientation_y +
                                    ego.orientation_z * ego.orientation_z))};
                const double lateral = cruiseObjectLateralDistance(
                    sampled, pose, shape, cruise_vehicle_.front_m,
                    cruise_vehicle_.rear_m, cruise_vehicle_.width_m,
                    ego_pose);
                if (!cruise_slowdown_candidates_.shouldSlowDown(
                        object.id, lateral)) continue;
                const auto interval = findCruiseSlowdownCollisionInterval(
                    sampled, pose, shape, cruise_vehicle_.front_m,
                    cruise_vehicle_.rear_m, cruise_vehicle_.width_m,
                    1.3, ego_pose);
                if (!interval) continue;
                cruise_slowdown_candidates_.accepted(object.id);
                slowdowns.push_back({
                    object.id,
                    dynamicStopArc(source, {interval->front_point.x,
                                            interval->front_point.y}),
                    dynamicStopArc(source, {interval->back_point.x,
                                            interval->back_point.y}),
                    lateral,
                    projection->longitudinal_velocity_mps,
                    projection->approach_velocity_mps});
              }
            }
            cruise_slowdown_candidates_.endFrame();
            last_slowdown_candidate_count_ = slowdowns.size();
            auto slowdown_result = cruise_slowdown_.plan(
                path, {ego.x, ego.y}, ego_speed_mps,
                cruise_acceleration_mps2_, slowdowns);
            last_slowdown_intervals_ = slowdown_result.intervals.slowdowns;
            return slowdown_result.trajectory;
          }) {
  if (!std::isfinite(route_backward_path_length_m_) ||
      route_backward_path_length_m_ < 0.0)
    throw std::invalid_argument("route backward path length must be nonnegative");
  modules_ = map_planner_->modules();
  auto areas = route_map_.costmapAreaPolygons();
  costmap_.onMapAreas(std::move(areas.road), std::move(areas.parking));
}

PlanningPipelineMapResult PlanningPipelineMapAdapter::onFrame(
    const PlanningPipelineFrame &frame,
    const BehaviorVelocityMapInput &velocity_context,
    const PlanningCruisePerceptionFrame &cruise_perception,
    const PlanningPipelinePointcloudInput &pointcloud,
    const std::optional<PlanningCruisePerceptionFrame> &cruise_callback_perception,
    const std::optional<PlanningOutOfLaneInput> &out_of_lane_input) {
  velocity_context_ = velocity_context;
  cruise_perception_ = cruise_callback_perception.value_or(cruise_perception);
  cruise_acceleration_mps2_ = frame.cruise_callback ?
      frame.cruise_callback->ego_acceleration_mps2.value_or(
          velocity_context.ego_acceleration_mps2) :
      velocity_context.ego_acceleration_mps2;
  pointcloud_ = pointcloud;
  out_of_lane_input_ = out_of_lane_input;
  last_velocity_result_.reset();
  last_slowdown_candidate_count_ = 0;
  last_slowdown_intervals_.clear();
  auto normalized = frame;
  normalized.ego.longitudinal_velocity_mps =
      static_cast<float>(frame.ego_speed_mps);
  auto result = pipeline_.onFrame(normalized);
  std::optional<VelocityLimit> selected_velocity_limit;
  std::optional<CruisePidVelocityLimit> pid_limit;
  if (result.status == PlanningPipelineStatus::Complete) {
    pid_limit = cruise_pid_.update(
        frame.cruise_callback ? frame.cruise_callback->ego_speed_mps :
                                frame.ego_speed_mps,
        cruise_candidates_.pidObstacles());
    const Stamp stamp{static_cast<std::int64_t>(
        std::llround((frame.cruise_callback ?
            frame.cruise_callback->timestamp_s : frame.timestamp_s) * 1.0e9))};
    if (pid_limit) {
      VelocityLimit limit;
      limit.stamp = stamp;
      limit.sender = "obstacle_cruise_planner.cruise";
      limit.max_velocity = static_cast<float>(pid_limit->maximum_velocity);
      limit.use_constraints = true;
      limit.constraints.min_acceleration = static_cast<float>(
          pid_limit->minimum_acceleration);
      limit.constraints.max_jerk = 1.0F;
      limit.constraints.min_jerk = -1.0F;
      selected_velocity_limit = velocity_limit_selector_.onInternalLimit(
          limit, stamp);
      cruise_limit_active_ = true;
    } else if (cruise_limit_active_) {
      selected_velocity_limit = velocity_limit_selector_.onClear(
          "obstacle_cruise_planner.cruise", true, stamp);
      cruise_limit_active_ = false;
    }
  }
  auto output = PlanningPipelineMapResult{
      std::move(result), std::move(last_velocity_result_),
          std::nullopt, std::nullopt, 0, {}, {}, {},
          {}, {}, last_slowdown_candidate_count_, std::nullopt, {}, {}, {},
          std::nullopt, 0, std::nullopt, {}, {}};
  output.selected_velocity_limit = std::move(selected_velocity_limit);
  output.cruise_pid_obstacle_count = cruise_candidates_.pidObstacles().size();
  output.cruise_pid_limit = std::move(pid_limit);
  output.cruise_diagnostics = cruise_candidates_.cruiseDiagnostics();
  output.cruise_slowdown_intervals = last_slowdown_intervals_;
  return output;
}

PlanningPipelineMapResult PlanningPipelineMapAdapter::onRouteFrame(
    PlanningPipelineFrame frame, const MissionRoute &route,
    const BehaviorVelocityMapInput &velocity_context,
    const PlanningCruisePerceptionFrame &cruise_perception,
    const PlanningPipelinePointcloudInput &pointcloud,
    std::optional<BehaviorGoalPose> goal,
    std::optional<BehaviorCandidateCallbackInput> candidates,
    double route_speed_mps,
    std::optional<BehaviorRouteActionInput> action,
    bool route_lane_change_activated,
    const std::optional<PlanningCruisePerceptionFrame> &cruise_callback_perception,
    const std::optional<PlanningOutOfLaneInput> &out_of_lane_input,
    PlanningRouteControlState route_control) {
  PlanningPipelineMapResult invalid;
  if (route.sections.empty() || !std::isfinite(route_speed_mps) ||
      route_speed_mps < 0.0) return invalid;
  // BehaviorPathPlannerNode::run calls resetCurrentRouteLanelet in manual
  // control unless an approved module can prevent rerouting. RouteHandler
  // then selects the closest route lane from the current pose again.
  if (!route_control.controlled_by_autoware_autonomously &&
      !route_control.has_possible_reroute_approved_module)
    current_route_lane_id_.reset();
  std::vector<std::int64_t> automatic_route_ids;
  if (!action) {
    std::vector<std::int64_t> candidates;
    for (const auto &section : route.sections)
      for (const auto id : section.primitive_lanelet_ids)
        if (std::find(candidates.begin(), candidates.end(), id) ==
            candidates.end()) candidates.push_back(id);
    const auto nearest = route_map_.closestRouteRoadLaneletId(
        frame.ego.x, frame.ego.y, pointYaw(frame.ego), candidates);
    std::optional<std::int64_t> next_lane;
    std::size_t source_section = route.sections.size();
    if (nearest) {
      auto lane_id = *nearest;
      double distance_to_lane_start = 0.0;
      for (std::size_t step = 0; step < route.sections.size(); ++step) {
        const auto section_it = std::find_if(
            route.sections.begin(), route.sections.end(),
            [&](const auto &section) {
              return std::find(section.primitive_lanelet_ids.begin(),
                               section.primitive_lanelet_ids.end(), lane_id) !=
                     section.primitive_lanelet_ids.end();
            });
        if (section_it == route.sections.end()) break;
        const auto section_index = static_cast<std::size_t>(
            section_it - route.sections.begin());
        if (lane_id != section_it->preferred_lanelet_id) {
          const auto target = route_map_.nextRoadLaneletTowardPreferred(
              lane_id, section_it->preferred_lanelet_id,
              section_it->primitive_lanelet_ids);
          // NormalLaneChange::isLaneChangeRequired checks the distance from
          // ego to target-lane start against max_prepare_duration * max_vel.
          const double maximum_speed = route_speed_mps > 0.0 ?
              route_speed_mps : route_map_.laneSpeedLimit(lane_id);
          if (target && distance_to_lane_start <= 4.0 * maximum_speed) {
            next_lane = target;
            source_section = section_index;
            break;
          }
        }
        const auto center = route_map_.laneCenterline(lane_id);
        if (center.size() < 2) break;
        double length = 0.0;
        for (std::size_t i = 1; i < center.size(); ++i)
          length += std::hypot(center[i].x - center[i - 1].x,
                               center[i].y - center[i - 1].y);
        if (step == 0) {
          double nearest_distance = std::numeric_limits<double>::infinity();
          double nearest_arc = 0.0;
          double arc = 0.0;
          for (std::size_t i = 1; i < center.size(); ++i) {
            const double dx = center[i].x - center[i - 1].x;
            const double dy = center[i].y - center[i - 1].y;
            const double span = std::hypot(dx, dy);
            if (span < 1.0e-9) continue;
            const double ratio = std::clamp(
                ((frame.ego.x - center[i - 1].x) * dx +
                 (frame.ego.y - center[i - 1].y) * dy) / (span * span),
                0.0, 1.0);
            const double distance = std::hypot(
                frame.ego.x - center[i - 1].x - ratio * dx,
                frame.ego.y - center[i - 1].y - ratio * dy);
            if (distance < nearest_distance) {
              nearest_distance = distance;
              nearest_arc = arc + ratio * span;
            }
            arc += span;
          }
          length = std::max(0.0, length - nearest_arc);
        }
        distance_to_lane_start += length;
        std::vector<std::int64_t> remaining;
        for (std::size_t i = section_index + 1; i < route.sections.size(); ++i)
          remaining.insert(remaining.end(),
                           route.sections[i].primitive_lanelet_ids.begin(),
                           route.sections[i].primitive_lanelet_ids.end());
        const auto following = route_map_.followingRoadLaneletId(
            lane_id, remaining);
        if (!following) break;
        lane_id = *following;
      }
    }
    if (next_lane) {
      // 현재 차로를 따라가면서 선호 차로와 그 후속 차로를 변경 목표로 잡습니다.
      automatic_route_ids.push_back(*nearest);
      std::vector<std::int64_t> target_ids{*next_lane};
      const auto current_section = std::find_if(
          route.sections.begin(), route.sections.end(),
          [&](const auto &section) {
            return std::find(section.primitive_lanelet_ids.begin(),
                             section.primitive_lanelet_ids.end(), *nearest) !=
                   section.primitive_lanelet_ids.end();
          });
      for (std::size_t index = static_cast<std::size_t>(
               current_section - route.sections.begin()) + 1;
           index < route.sections.size(); ++index) {
        const auto &next_section = route.sections[index];
        const auto current_next = route_map_.followingRoadLaneletId(
            automatic_route_ids.back(),
            next_section.primitive_lanelet_ids,
            next_section.preferred_lanelet_id);
        if (current_next && current_next != automatic_route_ids.back())
          automatic_route_ids.push_back(*current_next);
        if (index <= source_section) continue;
        const auto target_next = route_map_.followingRoadLaneletId(
            target_ids.back(), next_section.primitive_lanelet_ids,
            next_section.preferred_lanelet_id);
        if (target_next) target_ids.push_back(*target_next);
      }
      action = BehaviorRouteActionInput{};
      action->lane_change = BehaviorRouteLaneChangeRequest{};
      action->lane_change->target_lanelet_ids = std::move(target_ids);
      action->lane_change->sampling.enabled = true;
      action->lane_change->shift_length_m = 50.0;
      action->lane_change->execution.activated = route_lane_change_activated;
      action->lane_change->use_perception_objects = true;
    }
  }
  const bool nominal_route = !action;
  // The default lane-driving path must still be built from the route before
  // the static avoidance module evaluates perceived objects. The original
  // launch disables RTC for this module, so a feasible shift is approved
  // without a separate operator command.
  if (!action &&
      (!cruise_perception.objects.empty() || !registered_avoidance_.empty())) {
    action.emplace();
    action->avoidance.emplace();
    action->avoidance->approved = true;
    action->avoidance->use_perception_objects = true;
  }
  frame.behavior_path = {};
  frame.left_bound.clear();
  frame.right_bound.clear();
  std::vector<std::int64_t> route_ids = automatic_route_ids;
  if (nominal_route) {
    // RouteHandler::getLaneletSequence starts at the current route lanelet,
    // follows connected route primitives, and keeps the local sequence when
    // there is no successor. A route's preferred primitives need not form a
    // connected chain: a later lane change can lead to a different branch.
    std::vector<std::int64_t> signature;
    std::vector<std::int64_t> route_candidates;
    for (const auto &section : route.sections) {
      signature.push_back(section.preferred_lanelet_id);
      for (const auto id : section.primitive_lanelet_ids) {
        signature.push_back(id);
        if (std::find(route_candidates.begin(), route_candidates.end(), id) ==
            route_candidates.end()) route_candidates.push_back(id);
      }
      signature.push_back(0);
    }
    if (signature != route_signature_) {
      route_signature_ = std::move(signature);
      current_route_lane_id_.reset();
    }
    if (route_candidates.empty()) return invalid;
    const auto projected_arc = [&](std::int64_t id) {
      const auto center = route_map_.laneCenterline(id);
      double best = std::numeric_limits<double>::infinity();
      double best_arc = 0.0;
      double arc = 0.0;
      for (std::size_t i = 1; i < center.size(); ++i) {
        const double dx = center[i].x - center[i - 1].x;
        const double dy = center[i].y - center[i - 1].y;
        const double length = std::hypot(dx, dy);
        if (length < 1.0e-9) continue;
        const double ratio = std::clamp(
            ((frame.ego.x - center[i - 1].x) * dx +
             (frame.ego.y - center[i - 1].y) * dy) / (length * length),
            0.0, 1.0);
        const double distance = std::hypot(
            frame.ego.x - center[i - 1].x - ratio * dx,
            frame.ego.y - center[i - 1].y - ratio * dy);
        if (distance < best) {
          best = distance;
          best_arc = arc + ratio * length;
        }
        arc += length;
      }
      return best_arc;
    };
    const double ego_yaw = pointYaw(frame.ego);
    const auto closest = route_map_.closestRouteRoadLaneletId(
        frame.ego.x, frame.ego.y, ego_yaw, route_candidates);
    if (!closest) return invalid;
    std::int64_t current_id = *closest;
    if (current_route_lane_id_ &&
        std::find(route_candidates.begin(), route_candidates.end(),
                  *current_route_lane_id_) != route_candidates.end()) {
      // PlannerManager first searches the current route lanelet and its
      // preceding/following route neighbors with the configured 3m/1.046rad
      // pose constraints, then falls back to the closest route lanelet.
      std::vector<std::int64_t> neighbors;
      for (const auto id : route_candidates) {
        if (id != *current_route_lane_id_ &&
            route_map_.followingRoadLaneletId(
                id, {*current_route_lane_id_}) != *current_route_lane_id_ &&
            route_map_.followingRoadLaneletId(
                *current_route_lane_id_, {id}) != id) continue;
        neighbors.push_back(id);
      }
      const auto local = route_map_.closestRouteRoadLaneletId(
          frame.ego.x, frame.ego.y, ego_yaw, neighbors, 3.0, 1.046);
      if (local) current_id = *local;
    }
    current_route_lane_id_ = current_id;
    route_ids = {current_id};
    double forward_length = 0.0;
    while (forward_length < 300.0) {
      const auto next = route_map_.followingRoadLaneletId(
          route_ids.back(), route_candidates);
      if (!next || std::find(route_ids.begin(), route_ids.end(), *next) !=
                       route_ids.end()) break;
      const auto center = route_map_.laneCenterline(*next);
      for (std::size_t i = 1; i < center.size(); ++i) {
        forward_length += std::hypot(
            center[i].x - center[i - 1].x,
            center[i].y - center[i - 1].y);
      }
      route_ids.push_back(*next);
    }
    const double current_arc = projected_arc(current_id);
    double backward_length = current_arc;
    // getReferencePath requests an extra 10m of lanelets before its final
    // backward crop, so spline resampling stays stable near lane boundaries.
    while (backward_length < route_backward_path_length_m_ + 10.0) {
      const auto previous = std::find_if(
          route_candidates.begin(), route_candidates.end(),
          [&](std::int64_t id) {
            return std::find(route_ids.begin(), route_ids.end(), id) ==
                       route_ids.end() &&
                   route_map_.followingRoadLaneletId(
                       id, {route_ids.front()}) == route_ids.front();
          });
      if (previous == route_candidates.end()) break;
      const auto center = route_map_.laneCenterline(*previous);
      for (std::size_t i = 1; i < center.size(); ++i) {
        backward_length += std::hypot(
            center[i].x - center[i - 1].x,
            center[i].y - center[i - 1].y);
      }
      route_ids.insert(route_ids.begin(), *previous);
    }
  } else if (route_ids.empty()) {
    for (const auto &section : route.sections) {
      const auto lane_id = section.preferred_lanelet_id;
      if (lane_id == 0) return invalid;
      if (route_ids.empty() || route_ids.back() != lane_id)
        route_ids.push_back(lane_id);
    }
  }
  const double effective_route_speed_mps = route_speed_mps > 0.0 ?
      route_speed_mps : route_map_.laneSpeedLimit(route_ids.front());
  if (route_ids != active_route_ids_) {
    active_route_ids_ = route_ids;
    registered_avoidance_.clear();
    pull_over_phase_ = {};
    active_pull_over_action_.reset();
    active_pull_over_request_.reset();
    start_phase_ = {};
    active_start_action_.reset();
    active_start_request_.reset();
    active_side_shift_action_.reset();
    active_side_shift_request_.reset();
    last_side_shift_request_time_s_.reset();
    stationary_since_s_.clear();
    if (dynamic_avoidance_tracker_) dynamic_avoidance_tracker_->reset();
    dynamic_inside_last_time_s_.clear();
    dynamic_avoidance_last_objects_.clear();
    ego_stopped_since_s_.reset();
  }
  if (std::isfinite(frame.timestamp_s) &&
      std::isfinite(frame.ego_speed_mps) &&
      std::abs(frame.ego_speed_mps) <= 0.1) {
    if (!ego_stopped_since_s_ ||
        frame.timestamp_s < *ego_stopped_since_s_)
      ego_stopped_since_s_ = frame.timestamp_s;
  } else {
    ego_stopped_since_s_.reset();
  }
  std::vector<RouteLaneRange> route_lane_ranges;
  route_lane_ranges.reserve(route_ids.size());
  for (const auto lane_id : route_ids) {
    const auto centerline = route_map_.laneCenterline3d(lane_id);
    const auto bounds = route_map_.laneBounds(lane_id);
    if (centerline.size() < 2 || bounds.left.size() < 2 ||
        bounds.right.size() < 2) return invalid;
    if (!frame.behavior_path.points.empty() &&
        std::hypot(frame.behavior_path.points.back().x - centerline.front().x,
                   frame.behavior_path.points.back().y - centerline.front().y) >
            2.0) return invalid;
    const std::size_t first_point = frame.behavior_path.points.empty() ?
        0 : frame.behavior_path.points.size() - 1;
    for (const auto &point : centerline) {
      if (!frame.behavior_path.points.empty() &&
          std::hypot(frame.behavior_path.points.back().x - point.x,
                     frame.behavior_path.points.back().y - point.y) < 0.01) {
        auto &ids = frame.behavior_path.lane_ids.back();
        if (std::find(ids.begin(), ids.end(), lane_id) == ids.end())
          ids.push_back(lane_id);
        continue;
      }
      TrajectoryPoint trajectory_point;
      trajectory_point.x = point.x;
      trajectory_point.y = point.y;
      // RouteHandler::getCenterLinePath keeps the Lanelet2 centerline height.
      // The route goal height belongs to goal refinement, not every road point.
      trajectory_point.z = point.z;
      trajectory_point.longitudinal_velocity_mps =
          static_cast<float>(route_speed_mps > 0.0 ? route_speed_mps :
                             route_map_.laneSpeedLimit(lane_id));
      frame.behavior_path.points.push_back(trajectory_point);
      frame.behavior_path.lane_ids.push_back({lane_id});
    }
    route_lane_ranges.push_back(
        {first_point, frame.behavior_path.points.size() - 1});
    for (const auto &point : bounds.left)
      frame.left_bound.push_back({point.x, point.y});
    for (const auto &point : bounds.right)
      frame.right_bound.push_back({point.x, point.y});
  }
  if (frame.behavior_path.points.size() < 2) return invalid;
  const bool local_route_reaches_goal =
      route_ids.back() == route.sections.back().preferred_lanelet_id;
  frame.ego.longitudinal_velocity_mps =
      static_cast<float>(frame.ego_speed_mps);
  for (std::size_t index = 0;
       index < frame.behavior_path.points.size(); ++index) {
    const auto previous = index == 0 ? index : index - 1;
    const auto next = std::min(index + 1,
                               frame.behavior_path.points.size() - 1);
    const auto &a = frame.behavior_path.points[previous];
    const auto &b = frame.behavior_path.points[next];
    const double yaw = std::atan2(b.y - a.y, b.x - a.x);
    frame.behavior_path.points[index].orientation_z = std::sin(yaw / 2.0);
    frame.behavior_path.points[index].orientation_w = std::cos(yaw / 2.0);
  }
  BehaviorGoalPath route_reference_path;
  if (nominal_route || !automatic_route_ids.empty()) {
    auto reference_pose = frame.ego;
    if (experimental_route_sequence_clip_ && nominal_route) {
      const auto current = std::find(route_ids.begin(), route_ids.end(),
                                     *current_route_lane_id_);
      if (current != route_ids.end()) {
        const auto lane_index = static_cast<std::size_t>(
            current - route_ids.begin());
        reference_pose = projectEgoToRouteCenterline(
            frame.behavior_path, route_lane_ranges[lane_index], frame.ego);
      }
    }
    if (experimental_route_sequence_clip_)
      frame.behavior_path = clipRawRouteReference(
          frame.behavior_path, route_lane_ranges, reference_pose,
          route_backward_path_length_m_,
          local_route_reaches_goal ? goal : std::nullopt, route_ids.back());
    frame.behavior_path = resampleRouteReference(frame.behavior_path, false);
    if (goal && local_route_reaches_goal && !experimental_route_sequence_clip_)
      frame.behavior_path = truncateRouteAtGoal(
          frame.behavior_path, *goal, route_ids.back());
    frame.behavior_path = cropRouteReference(
        frame.behavior_path, reference_pose, route_backward_path_length_m_,
        experimental_route_sequence_clip_);
    route_reference_path = frame.behavior_path;
  }
  if (goal && !(action && action->pull_over)) {
    frame.behavior_path = route_map_.refineFixedGoalPath(
        frame.behavior_path, *goal,
        route.sections.back().preferred_lanelet_id);
    frame.goal_contained = local_route_reaches_goal;
  }
  BehaviorGoalPath route_goal_refined_path;
  if (nominal_route || !automatic_route_ids.empty())
    route_goal_refined_path = frame.behavior_path;
  std::optional<CrosswalkOccupancyGrid> generated_parking_grid;
  const bool needs_freespace_grid = action &&
      ((action->start &&
        (action->start->use_freespace ||
         action->start->use_freespace_fallback) &&
        !action->start->freespace_grid) ||
       (action->pull_over &&
        (action->pull_over->use_freespace ||
         action->pull_over->use_freespace_fallback) &&
        !action->pull_over->freespace_grid));
  if (needs_freespace_grid) {
    CostmapGeneratorInput grid_input;
    grid_input.stamp_s = frame.timestamp_s;
    grid_input.ego_x = frame.ego.x;
    grid_input.ego_y = frame.ego.y;
    grid_input.ego_z = frame.ego.z;
    grid_input.ego_in_parking_lot = route_map_.scenarioPosition(
        frame.ego.x, frame.ego.y, 0.0,
        goal ? goal->x : frame.ego.x,
        goal ? goal->y : frame.ego.y,
        route_ids).in_parking_lot;
    for (const auto &point : pointcloud.cruise_points)
      grid_input.points_map_frame.push_back(
          {point.x, point.y, point.z});
    const double grid_object_elapsed_s =
        cruise_perception.stamp_s > 0.0 ?
            std::max(0.0, frame.timestamp_s -
                              cruise_perception.stamp_s) : 0.0;
    for (const auto &object : cruise_perception.objects) {
      if (!std::isfinite(object.x) || !std::isfinite(object.y) ||
          !std::isfinite(object.yaw_rad) ||
          !std::isfinite(object.length_m) ||
          !std::isfinite(object.width_m) ||
          object.length_m <= 0.0 || object.width_m <= 0.0)
        continue;
      BehaviorSafetyObject shape;
      shape.shape_type = object.shape_type == 2 &&
          object.footprint.empty() ? 0 : object.shape_type;
      shape.length_m = object.length_m;
      shape.width_m = object.width_m;
      for (const auto &point : object.footprint)
        shape.footprint.emplace_back(point.x, point.y);
      const auto pose = currentCruiseObjectPose(
          {object.x, object.y, object.yaw_rad},
          object.predicted_paths, grid_object_elapsed_s);
      grid_input.objects_map_frame.push_back({
          behaviorSafetyWorldPolygon(
              {0.0, pose.x, pose.y, pose.yaw_rad, 0.0}, shape),
          1.0, object.shape_type, pose.x, pose.y, pose.yaw_rad,
          object.length_m, object.width_m});
    }
    if (auto generated = costmap_.tick(grid_input)) {
      generated_parking_grid = std::move(generated->occupancy);
      if (action->start && !action->start->freespace_grid)
        action->start->freespace_grid = generated_parking_grid;
      if (action->pull_over && !action->pull_over->freespace_grid)
        action->pull_over->freespace_grid = generated_parking_grid;
    }
  }
  bool stale_start_perception = false;
  if (action && action->start && action->start->use_ego_pose &&
      !candidates) {
    const double perception_time = cruise_perception.stamp_s > 0.0 ?
        cruise_perception.stamp_s : frame.timestamp_s;
    const double age_s = frame.timestamp_s - perception_time;
    if (!std::isfinite(age_s) || age_s < 0.0 || age_s > 2.0 ||
        frame.timestamp_s < 0.0 || frame.timestamp_s > 9.0e9) {
      stale_start_perception = true;
    } else {
      candidates.emplace();
      candidates->callback_time_ns = static_cast<std::int64_t>(
          std::llround(frame.timestamp_s * 1.0e9));
      candidates->perception.receipt_time_ns =
          static_cast<std::int64_t>(
              std::llround(perception_time * 1.0e9));
      candidates->maximum_perception_age_ns = 2000000000LL;
      for (const auto &object : cruise_perception.objects) {
        if (!std::isfinite(object.x) || !std::isfinite(object.y) ||
            !std::isfinite(object.yaw_rad) ||
            !std::isfinite(object.length_m) ||
            !std::isfinite(object.width_m) ||
            !std::isfinite(object.velocity_x_mps) ||
            !std::isfinite(object.velocity_y_mps) ||
            object.length_m <= 0.0 || object.width_m <= 0.0)
          continue;
        const auto current = currentCruiseObjectPose(
            {object.x, object.y, object.yaw_rad},
            object.predicted_paths, age_s);
        if (!std::isfinite(current.x) || !std::isfinite(current.y) ||
            !std::isfinite(current.yaw_rad)) continue;
        StartPullOutStopObject converted;
        converted.pose.x = current.x;
        converted.pose.y = current.y;
        converted.pose.yaw_rad = current.yaw_rad;
        converted.shape.shape_type = object.shape_type == 2 &&
            object.footprint.empty() ? 0 : object.shape_type;
        converted.shape.length_m = object.length_m;
        converted.shape.width_m = object.width_m;
        for (const auto &point : object.footprint)
          converted.shape.footprint.emplace_back(point.x, point.y);
        converted.velocity_x_mps = object.velocity_x_mps;
        converted.velocity_y_mps = object.velocity_y_mps;
        converted.allowed_class = object.classification <
            action->start->object_types_to_check.size() &&
            action->start->object_types_to_check[object.classification];
        candidates->perception.objects.push_back(std::move(converted));
      }
    }
  }
  std::optional<BehaviorCandidateCallbackResult> candidate_result;
  if (candidates) {
    std::int64_t start_lane_id = route_ids.front();
    double nearest_lane_distance = std::numeric_limits<double>::infinity();
    for (const auto lane_id : route_ids) {
      const auto centerline = route_map_.laneCenterline(lane_id);
      for (const auto &point : centerline) {
        const double distance = std::hypot(
            frame.ego.x - point.x, frame.ego.y - point.y);
        if (distance < nearest_lane_distance) {
          nearest_lane_distance = distance;
          start_lane_id = lane_id;
        }
      }
    }
    const auto start_centerline = route_map_.laneCenterline(start_lane_id);
    candidates->start.centerline.clear();
    for (const auto &point : start_centerline)
      candidates->start.centerline.push_back({point.x, point.y});
    double accumulated = 0.0;
    double nearest_distance = std::numeric_limits<double>::infinity();
    candidates->start.start_lane_arc_m = 0.0;
    for (std::size_t index = 0; index + 1 < start_centerline.size(); ++index) {
      const auto &a = start_centerline[index];
      const auto &b = start_centerline[index + 1];
      const double dx = b.x - a.x;
      const double dy = b.y - a.y;
      const double length = std::hypot(dx, dy);
      if (length < 1.0e-9) continue;
      const double ratio = std::clamp(
          ((frame.ego.x - a.x) * dx + (frame.ego.y - a.y) * dy) /
              (length * length), 0.0, 1.0);
      const double distance = std::hypot(
          frame.ego.x - a.x - ratio * dx,
          frame.ego.y - a.y - ratio * dy);
      if (distance < nearest_distance) {
        nearest_distance = distance;
        candidates->start.start_lane_arc_m = accumulated + ratio * length;
      }
      accumulated += length;
    }
    candidates->start.lane_length_m = accumulated;
    candidates->start.lane_polygons = route_map_.lanePolygons(route_ids);
    candidates->start.original_start_pose = {
        0.0, frame.ego.x, frame.ego.y,
        std::atan2(2.0 * (frame.ego.orientation_w *
                             frame.ego.orientation_z +
                         frame.ego.orientation_x *
                             frame.ego.orientation_y),
                   1.0 - 2.0 * (frame.ego.orientation_y *
                                    frame.ego.orientation_y +
                                frame.ego.orientation_z *
                                    frame.ego.orientation_z)),
        frame.ego_speed_mps};
    if (action && action->start) {
      candidates->start.vehicle.front_m = action->start->vehicle.front_m;
      candidates->start.vehicle.rear_m = action->start->vehicle.back_m;
      candidates->start.vehicle.half_width_m =
          action->start->vehicle.half_width_m;
    }
    if (goal) candidates->goal.goal = *goal;
    if (action && action->pull_over &&
        candidates->goal.object_hard_margin_m == 0.0)
      candidates->goal.object_hard_margin_m =
          action->pull_over->object_hard_margin_m;
    candidate_result = evaluateBehaviorCandidateCallback(*candidates);
  }
  std::optional<BehaviorRouteActionResult> action_result;
  std::size_t avoidance_perception_input_count = 0;
  std::vector<BehaviorDynamicAvoidanceObservation>
      dynamic_avoidance_targets;
  std::vector<BehaviorDynamicAvoidanceObservation>
      dynamic_avoidance_final_targets;
  std::vector<BehaviorDynamicAvoidanceObservation>
      dynamic_avoidance_selected_targets;
  std::vector<BehaviorSafetyPolygon>
      dynamic_avoidance_candidate_polygons;
  std::vector<bool> dynamic_avoidance_collision_on_left;
  if (action && action->dynamic_avoidance &&
      behaviorDynamicPathForward(frame.behavior_path.points)) {
    const auto &parameters =
        action->dynamic_avoidance->parameters;
    const bool changed = !dynamic_avoidance_parameters_ ||
        dynamic_avoidance_parameters_->avoid_types !=
            parameters.avoid_types ||
        dynamic_avoidance_parameters_->minimum_speed_mps !=
            parameters.minimum_speed_mps ||
        dynamic_avoidance_parameters_->maximum_speed_mps !=
            parameters.maximum_speed_mps ||
        dynamic_avoidance_parameters_->successive_entry_count !=
            parameters.successive_entry_count ||
        dynamic_avoidance_parameters_->successive_exit_count !=
            parameters.successive_exit_count ||
        dynamic_avoidance_parameters_
            ->maximum_object_lateral_distance_m !=
            parameters.maximum_object_lateral_distance_m ||
        dynamic_avoidance_parameters_
            ->maximum_overtaking_crossing_angle_rad !=
            parameters.maximum_overtaking_crossing_angle_rad ||
        dynamic_avoidance_parameters_
            ->maximum_oncoming_crossing_angle_rad !=
            parameters.maximum_oncoming_crossing_angle_rad ||
        dynamic_avoidance_parameters_
            ->minimum_overtaking_crossing_speed_mps !=
            parameters.minimum_overtaking_crossing_speed_mps ||
        dynamic_avoidance_parameters_
            ->minimum_oncoming_crossing_speed_mps !=
            parameters.minimum_oncoming_crossing_speed_mps ||
        dynamic_avoidance_parameters_
            ->maximum_pedestrian_lateral_speed_mps !=
            parameters.maximum_pedestrian_lateral_speed_mps ||
        dynamic_avoidance_parameters_->maximum_front_object_angle_rad !=
            parameters.maximum_front_object_angle_rad ||
        dynamic_avoidance_parameters_->minimum_front_object_speed_mps !=
            parameters.minimum_front_object_speed_mps ||
        dynamic_avoidance_parameters_
            ->maximum_moving_object_collision_time_s !=
            parameters.maximum_moving_object_collision_time_s ||
        dynamic_avoidance_parameters_->minimum_cut_in_object_speed_mps !=
            parameters.minimum_cut_in_object_speed_mps ||
        dynamic_avoidance_parameters_->minimum_cut_in_start_time_s !=
            parameters.minimum_cut_in_start_time_s ||
        dynamic_avoidance_parameters_
            ->minimum_cut_in_longitudinal_distance_m !=
            parameters.minimum_cut_in_longitudinal_distance_m ||
        dynamic_avoidance_parameters_->minimum_cut_out_object_speed_mps !=
            parameters.minimum_cut_out_object_speed_mps ||
        dynamic_avoidance_parameters_->minimum_cut_out_lateral_speed_mps !=
            parameters.minimum_cut_out_lateral_speed_mps ||
        dynamic_avoidance_parameters_->maximum_cut_out_since_inside_s !=
            parameters.maximum_cut_out_since_inside_s;
    if (changed || !dynamic_avoidance_tracker_) {
      dynamic_avoidance_tracker_.emplace(parameters);
      dynamic_avoidance_parameters_ = parameters;
      dynamic_inside_last_time_s_.clear();
      dynamic_avoidance_last_objects_.clear();
      dynamic_avoidance_feasibility_origin_.clear();
      dynamic_avoidance_previous_boundary_.clear();
    }
    std::vector<BehaviorDynamicAvoidanceObservation> observed;
    std::vector<std::string> currently_inside_ids;
    observed.reserve(cruise_perception.objects.size());
    for (const auto &object : cruise_perception.objects) {
      // 원본 동적 회피는 예측 보간값이 아닌 인식 초기 자세로 대상을 고른다.
      const CruisePredictedPose pose{
          object.x, object.y, object.yaw_rad};
      TrajectoryPoint object_point;
      object_point.x = pose.x;
      object_point.y = pose.y;
      const auto nearest = nearestIndex(
          frame.behavior_path.points, object_point);
      const double yaw_difference = std::remainder(
          pose.yaw_rad - pointYaw(
              frame.behavior_path.points[nearest]),
          2.0 * std::acos(-1.0));
      const double tangent_speed =
          std::cos(yaw_difference) * object.velocity_x_mps -
          std::sin(yaw_difference) * object.velocity_y_mps;
      const double normal_speed =
          std::sin(yaw_difference) * object.velocity_x_mps +
          std::cos(yaw_difference) * object.velocity_y_mps;
      double radius = std::hypot(
          object.length_m * 0.5, object.width_m * 0.5);
      if (object.shape_type == 1)
        radius = object.length_m * 0.5;
      else if (object.shape_type == 2) {
        radius = 0.0;
        for (const auto &point : object.footprint)
          radius = std::max(radius,
                            std::hypot(point.x, point.y));
      }
      if (!behaviorDynamicRoughAdmission(
              {object.classification, tangent_speed, normal_speed,
               std::hypot(object.velocity_x_mps,
                          object.velocity_y_mps),
               yaw_difference,
               std::abs(lateralOffset(
                   frame.behavior_path.points, object_point)),
               radius, cruise_vehicle_.width_m * 0.5}, parameters))
        continue;
      observed.push_back({
          object.id, object.classification,
          std::hypot(object.velocity_x_mps,
                     object.velocity_y_mps),
          pose.x, pose.y});
      dynamic_avoidance_last_objects_[object.id] = object;
      if (std::abs(lateralOffset(
              frame.behavior_path.points, object_point)) <
              cruise_vehicle_.width_m * 0.5)
        currently_inside_ids.push_back(object.id);
    }
    dynamic_avoidance_targets =
        dynamic_avoidance_tracker_->advance(observed);
    std::unordered_set<std::string> retained_ids;
    for (const auto &item : observed) retained_ids.insert(item.id);
    for (const auto &item : dynamic_avoidance_targets)
      retained_ids.insert(item.id);
    for (auto item = dynamic_avoidance_last_objects_.begin();
         item != dynamic_avoidance_last_objects_.end();) {
      if (retained_ids.count(item->first) == 0)
        item = dynamic_avoidance_last_objects_.erase(item);
      else ++item;
    }
    std::unordered_set<std::string> valid_target_ids;
    for (const auto &target : dynamic_avoidance_targets)
      valid_target_ids.insert(target.id);
    for (auto item = dynamic_avoidance_feasibility_origin_.begin();
         item != dynamic_avoidance_feasibility_origin_.end();) {
      if (valid_target_ids.count(item->first) == 0)
        item = dynamic_avoidance_feasibility_origin_.erase(item);
      else ++item;
    }
    for (auto item = dynamic_avoidance_previous_boundary_.begin();
         item != dynamic_avoidance_previous_boundary_.end();) {
      if (valid_target_ids.count(item->first) == 0)
        item = dynamic_avoidance_previous_boundary_.erase(item);
      else ++item;
    }
    for (const auto &target : dynamic_avoidance_targets) {
      const auto [feasibility_entry, newly_valid] =
          dynamic_avoidance_feasibility_origin_.try_emplace(
              target.id,
              BehaviorSafetyPose{frame.timestamp_s,
                                 frame.ego.x, frame.ego.y,
                                 pointYaw(frame.ego),
                                 frame.ego_speed_mps});
      (void)newly_valid;
      const auto last_object = dynamic_avoidance_last_objects_.find(
          target.id);
      if (last_object == dynamic_avoidance_last_objects_.end()) continue;
      const PlanningPerceivedObject *object = &last_object->second;
      const CruisePredictedPose current{
          object->x, object->y, object->yaw_rad};
      TrajectoryPoint object_point;
      object_point.x = current.x;
      object_point.y = current.y;
      const auto nearest = nearestIndex(
          frame.behavior_path.points, object_point);
      const double yaw_difference = std::remainder(
          current.yaw_rad - pointYaw(
              frame.behavior_path.points[nearest]),
          2.0 * std::acos(-1.0));
      const double tangent_speed =
          std::cos(yaw_difference) * object->velocity_x_mps -
          std::sin(yaw_difference) * object->velocity_y_mps;
      const double normal_speed =
          std::sin(yaw_difference) * object->velocity_x_mps +
          std::cos(yaw_difference) * object->velocity_y_mps;
      const bool on_left = lateralOffset(
          frame.behavior_path.points, object_point) > 0.0;
      BehaviorSafetyObject shape;
      shape.shape_type = object->shape_type < 0 ||
          object->shape_type > 2 ||
          (object->shape_type == 2 && object->footprint.empty()) ?
          0 : object->shape_type;
      shape.length_m = object->length_m;
      shape.width_m = object->width_m;
      for (const auto &point : object->footprint)
        shape.footprint.emplace_back(point.x, point.y);
      const auto object_polygon = behaviorSafetyWorldPolygon(
          {0.0, current.x, current.y, current.yaw_rad, 0.0}, shape);
      const auto object_offsets = behaviorDynamicPathOffsets(
          frame.behavior_path.points, current.x, current.y,
          object_polygon);
      if (!object_offsets) continue;
      const double object_arc = object_offsets->object_arc_m;
      const double ego_arc = longitudinalArc(
          frame.behavior_path.points, frame.ego.x, frame.ego.y);
      if (object_arc + object_offsets->maximum_longitudinal_m <
          ego_arc - cruise_vehicle_.rear_m)
        continue;
      const auto object_type = behaviorDynamicObjectType(
          object->classification, parameters);
      bool collision_on_left = on_left;
      if (object_type == BehaviorDynamicObjectType::Regulated) {
        const bool aligned =
            std::abs(yaw_difference) <
                parameters.maximum_front_object_angle_rad ||
            std::abs(yaw_difference) >
                std::acos(-1.0) -
                    parameters.maximum_front_object_angle_rad;
        const bool on_ego_path = std::abs(lateralOffset(
            frame.behavior_path.points, object_point)) <
            cruise_vehicle_.width_m * 0.5;
        if (on_ego_path && aligned &&
            tangent_speed > parameters.minimum_front_object_speed_mps)
          continue;
        bool enters_ego_corridor = false;
        if (!object->predicted_paths.empty()) {
          const auto strongest = std::max_element(
              object->predicted_paths.begin(),
              object->predicted_paths.end(),
              [](const auto &a, const auto &b) {
                return a.confidence < b.confidence;
              });
          for (const auto &pose : strongest->poses) {
            TrajectoryPoint predicted;
            predicted.x = pose.x;
            predicted.y = pose.y;
            if (std::abs(lateralOffset(
                    frame.behavior_path.points, predicted)) <
                cruise_vehicle_.width_m * 0.5) {
              enters_ego_corridor = true;
              break;
            }
          }
        }
        if (behaviorDynamicObjectWillCutIn({
                enters_ego_corridor, tangent_speed,
                frame.ego_speed_mps,
                object_arc - ego_arc +
                    object_offsets->minimum_longitudinal_m,
                parameters.minimum_cut_in_object_speed_mps,
                parameters.minimum_cut_in_longitudinal_distance_m,
                parameters.minimum_cut_in_start_time_s}))
          continue;
        std::optional<double> since_inside;
        const auto previous_inside = dynamic_inside_last_time_s_.find(
            object->id);
        if (previous_inside != dynamic_inside_last_time_s_.end())
          since_inside = std::max(
              0.0, frame.timestamp_s - previous_inside->second);
        if (behaviorDynamicObjectWillCutOut({
                on_left, tangent_speed, normal_speed, since_inside,
                parameters.minimum_cut_out_object_speed_mps,
                parameters.minimum_cut_out_lateral_speed_mps,
                parameters.maximum_cut_out_since_inside_s}))
          continue;
        const auto collision_time = behaviorDynamicCollisionTiming({
            frame.ego_speed_mps, tangent_speed,
            object_arc - ego_arc,
            object_offsets->minimum_longitudinal_m,
            object_offsets->maximum_longitudinal_m,
            cruise_vehicle_.front_m,
            cruise_vehicle_.rear_m});
        if (std::hypot(object->velocity_x_mps,
                       object->velocity_y_mps) > 0.5 &&
            collision_time.start_s >
                parameters.maximum_moving_object_collision_time_s)
          continue;
        if (tangent_speed <= 0.0 &&
            !object->predicted_paths.empty() &&
            std::isfinite(collision_time.start_s) &&
            collision_time.start_s < 1.0e6) {
          const auto future = currentCruiseObjectPose(
              {object->x, object->y, object->yaw_rad},
              object->predicted_paths,
              collision_time.start_s);
          TrajectoryPoint future_point;
          future_point.x = future.x;
          future_point.y = future.y;
          collision_on_left = lateralOffset(
              frame.behavior_path.points, future_point) > 0.0;
        }
      }
      if (behaviorDynamicObjectType(
              object->classification, parameters) ==
          BehaviorDynamicObjectType::Regulated) {
        BehaviorDynamicRegulatedPlanInput plan;
        plan.reference = frame.behavior_path.points;
        plan.ego_x = frame.ego.x;
        plan.ego_y = frame.ego.y;
        plan.ego_yaw_rad = pointYaw(frame.ego);
        plan.ego_speed_mps = frame.ego_speed_mps;
        plan.ego_front_m = cruise_vehicle_.front_m;
        plan.ego_rear_m = cruise_vehicle_.rear_m;
        plan.vehicle_half_width_m =
            cruise_vehicle_.width_m * 0.5;
        plan.feasibility_origin = feasibility_entry->second;
        plan.object_x = current.x;
        plan.object_y = current.y;
        plan.object_path_speed_mps = tangent_speed;
        plan.object_lateral_speed_mps = normal_speed;
        plan.object_polygon = object_polygon;
        plan.collision_on_left = collision_on_left;
        if (const auto previous =
                dynamic_avoidance_previous_boundary_.find(target.id);
            previous != dynamic_avoidance_previous_boundary_.end()) {
          plan.previous_near_m = previous->second.near_m;
          if (previous->second.reference.size() >= 2) {
            const auto &current_reference_point =
                frame.behavior_path.points[nearest];
            plan.use_lateral_lowpass = std::abs(lateralOffset(
                previous->second.reference,
                current_reference_point)) < 0.3;
          }
        }
        BehaviorDynamicRegulatedLateralInput final_lateral;
        final_lateral.object_min_lateral_m =
            object_offsets->minimum_lateral_m;
        final_lateral.object_max_lateral_m =
            object_offsets->maximum_lateral_m;
        final_lateral.object_path_speed_mps = tangent_speed;
        final_lateral.object_lateral_speed_mps = normal_speed;
        final_lateral.collision_on_left = collision_on_left;
        final_lateral.vehicle_half_width_m =
            plan.vehicle_half_width_m;
        final_lateral.previous_near_m = plan.previous_near_m;
        final_lateral.use_lowpass = plan.use_lateral_lowpass;
        const auto final_lateral_bounds =
            behaviorDynamicRegulatedLateralBounds(final_lateral);
        if (!final_lateral_bounds) continue;
        dynamic_avoidance_final_targets.push_back(target);
        dynamic_avoidance_previous_boundary_[target.id] = {
            final_lateral_bounds->near_m,
            frame.behavior_path.points};
        if (const auto candidate =
                planBehaviorDynamicRegulatedObject(plan)) {
          dynamic_avoidance_candidate_polygons.push_back(
              candidate->exclusion_polygon);
          dynamic_avoidance_collision_on_left.push_back(
              collision_on_left);
          dynamic_avoidance_selected_targets.push_back(target);
        }
      } else {
        BehaviorDynamicUnregulatedLateralInput lateral;
        lateral.object_min_lateral_m =
            object_offsets->minimum_lateral_m;
        lateral.object_max_lateral_m =
            object_offsets->maximum_lateral_m;
        if (shape.shape_type == 1) {
          const double center_lateral = lateralOffset(
              frame.behavior_path.points, object_point);
          const double radius = object->length_m * 0.5;
          const double nearest_edge =
              std::abs(center_lateral) < radius ? 0.0 :
              center_lateral > 0.0 ?
                  center_lateral - radius :
                  center_lateral + radius;
          lateral.object_min_lateral_m = nearest_edge;
          lateral.object_max_lateral_m = nearest_edge;
        }
        lateral.object_lateral_speed_mps = normal_speed;
        lateral.vehicle_half_width_m =
            cruise_vehicle_.width_m * 0.5;
        if (const auto previous =
                dynamic_avoidance_previous_boundary_.find(target.id);
            previous != dynamic_avoidance_previous_boundary_.end()) {
          lateral.previous_near_m = previous->second.near_m;
          if (previous->second.reference.size() >= 2) {
            const auto &current_reference_point =
                frame.behavior_path.points[nearest];
            lateral.use_lowpass = std::abs(lateralOffset(
                previous->second.reference,
                current_reference_point)) < 0.3;
          }
        }
        const auto lateral_bounds =
            behaviorDynamicUnregulatedLateralBounds(lateral);
        if (!lateral_bounds) continue;
        dynamic_avoidance_final_targets.push_back(target);
        dynamic_avoidance_previous_boundary_[target.id] = {
            lateral_bounds->near_m,
            frame.behavior_path.points};
        const bool unregulated_collision_on_left =
            lateral_bounds->far_m > 0.0;
        BehaviorDynamicPredictedPolygonInput plan;
        plan.object_length_m = object->length_m;
        plan.object_width_m = object->width_m;
        for (const auto &path : object->predicted_paths) {
          BehaviorDynamicPredictedPath converted;
          converted.confidence = path.confidence;
          converted.time_step_s = path.time_step_s;
          for (std::size_t index = 0;
               index < path.poses.size(); ++index) {
            const auto &pose = path.poses[index];
            converted.poses.push_back({
                index * path.time_step_s,
                pose.x, pose.y, pose.yaw_rad, 0.0});
          }
          plan.paths.push_back(std::move(converted));
        }
        const auto reserve = behaviorDynamicEgoReservePolygon(
            frame.behavior_path.points,
            unregulated_collision_on_left,
            cruise_vehicle_.width_m * 0.5);
        if (reserve) plan.ego_reserve_polygon = *reserve;
        if (const auto polygon =
                behaviorDynamicPredictedPolygon(plan)) {
          dynamic_avoidance_candidate_polygons.push_back(
              *polygon);
          dynamic_avoidance_collision_on_left.push_back(
              unregulated_collision_on_left);
          dynamic_avoidance_selected_targets.push_back(target);
        }
      }
    }
    for (const auto &id : currently_inside_ids)
      dynamic_inside_last_time_s_[id] = frame.timestamp_s;
  }
  if (action || !registered_avoidance_.empty() ||
      (active_lane_change_action_ && route_lane_change_activated)) {
    auto request = action.value_or(BehaviorRouteActionInput{});
    if (request.sampling &&
        request.sampling->previous_path.empty())
      request.sampling->previous_path = active_sampling_path_;
    const bool temporary_avoidance_input = request.lane_change &&
        request.lane_change->avoidance_activation &&
        request.lane_change->avoidance_activation->objects.empty() &&
        !request.avoidance;
    if (temporary_avoidance_input) {
      request.avoidance = BehaviorRouteAvoidanceRequest{};
      request.avoidance->use_perception_objects = true;
    }
    if (request.start && request.start->use_ego_pose) {
      request.start->start = {frame.ego.x, frame.ego.y};
      request.start->lateral_offset_m = lateralOffset(
          frame.behavior_path.points, frame.ego);
      request.start->terminal_is_goal = goal.has_value();
    }
    if (request.side_shift && request.side_shift->use_ego_offset)
      request.side_shift->current_lateral_offset_m = lateralOffset(
          frame.behavior_path.points, frame.ego);
    if (stale_start_perception) request.start.reset();
    if (request.start && request.start->use_ego_pose &&
        candidate_result) {
      request.start->start_pose_candidates =
          candidate_result->start_candidates;
      if (request.start->start_pose_candidates.empty())
        request.start.reset();
    }
    if (request.start) {
      if (candidates) {
        for (const auto &object : candidates->perception.objects) {
          if (!object.allowed_class ||
              std::hypot(object.velocity_x_mps,
                         object.velocity_y_mps) >= 1.0)
            continue;
          request.start->path_stop_objects.push_back(
              behaviorSafetyWorldPolygon(object.pose, object.shape));
        }
      } else {
        const double elapsed_s = cruise_perception.stamp_s > 0.0 ?
            std::max(0.0, frame.timestamp_s -
                              cruise_perception.stamp_s) : 0.0;
        for (const auto &object : cruise_perception.objects) {
          if (object.classification >=
                  request.start->object_types_to_check.size() ||
              !request.start->object_types_to_check[object.classification] ||
              object.length_m <= 0.0 || object.width_m <= 0.0 ||
              std::hypot(object.velocity_x_mps,
                         object.velocity_y_mps) >= 1.0)
            continue;
          BehaviorSafetyObject shape;
          shape.shape_type = object.shape_type < 0 ||
              object.shape_type > 2 ||
              (object.shape_type == 2 && object.footprint.empty()) ?
              0 : object.shape_type;
          shape.length_m = object.length_m;
          shape.width_m = object.width_m;
          for (const auto &point : object.footprint)
            shape.footprint.emplace_back(point.x, point.y);
          const auto pose = currentCruiseObjectPose(
              {object.x, object.y, object.yaw_rad},
              object.predicted_paths, elapsed_s);
          request.start->path_stop_objects.push_back(
              behaviorSafetyWorldPolygon(
                  {0.0, pose.x, pose.y, pose.yaw_rad, 0.0}, shape));
        }
      }
    }
    if (request.avoidance && request.avoidance->use_ego_pose) {
      request.avoidance->ego_linear_shift_m = lateralOffset(
          frame.behavior_path.points, frame.ego);
    }
    if (!request.avoidance && !registered_avoidance_.empty()) {
      request.avoidance = BehaviorRouteAvoidanceRequest{};
      request.avoidance->ego_linear_shift_m = lateralOffset(
          frame.behavior_path.points, frame.ego);
    }
    if (request.avoidance &&
        request.avoidance->use_perception_objects) {
      auto parameters =
          request.avoidance->perception_parameters;
      if (goal && !parameters.goal_distance_from_ego_m) {
        parameters.goal_distance_from_ego_m =
            longitudinalArc(frame.behavior_path.points,
                            goal->x, goal->y) -
            longitudinalArc(frame.behavior_path.points,
                            frame.ego.x, frame.ego.y);
      }
      const double compensation_s =
          std::isfinite(parameters.lost_object_compensation_s) ?
          std::max(0.0, parameters.lost_object_compensation_s) : 0.0;
      const double now_s = cruise_perception.stamp_s > 0.0 ?
          cruise_perception.stamp_s : frame.timestamp_s;
      std::unordered_set<std::string> seen;
      std::vector<BehaviorRouteObservedObject> observations;
      for (const auto &object : cruise_perception.objects) {
        if (object.id.empty() || !std::isfinite(object.x) ||
            !std::isfinite(object.y) ||
            !std::isfinite(object.yaw_rad) ||
            !std::isfinite(object.length_m) ||
            !std::isfinite(object.width_m) ||
            object.length_m <= 0.0 || object.width_m <= 0.0)
          continue;
        seen.insert(object.id);
        const double speed = std::hypot(
            object.velocity_x_mps, object.velocity_y_mps);
        if (!std::isfinite(speed) ||
            speed > behaviorRouteAvoidanceSpeedThreshold(
                parameters, object.classification) ||
            !std::isfinite(now_s)) {
          stationary_since_s_.erase(object.id);
          continue;
        }
        auto [it, inserted] = stationary_since_s_.emplace(
            object.id, StationaryObservationState{
                now_s, now_s, object.x, object.y});
        if (!inserted &&
            (now_s < it->second.since_s ||
             now_s - it->second.last_seen_s >
                 compensation_s)) {
          it->second = {now_s, now_s, object.x, object.y};
        }
        it->second.last_seen_s = now_s;
        BehaviorRouteObservedObject observed;
        observed.x = object.x;
        observed.y = object.y;
        observed.yaw_rad = object.yaw_rad;
        observed.length_m = object.length_m;
        observed.width_m = object.width_m;
        observed.speed_mps = speed;
        observed.stationary_duration_s = std::max(
            0.0, now_s - it->second.since_s);
        observed.initial_displacement_m = std::hypot(
            object.x - it->second.x,
            object.y - it->second.y);
        observed.classification = object.classification;
        observed.within_crosswalk = route_map_.pointInCrosswalk(
            object.x, object.y);
        observed.within_intersection = route_map_.pointInIntersection(
            object.x, object.y);
        TrajectoryPoint object_pose;
        object_pose.x = object.x;
        object_pose.y = object.y;
        const bool object_on_left = lateralOffset(
            frame.behavior_path.points, object_pose) > 0.0;
        BehaviorSafetyObject object_shape;
        object_shape.shape_type = object.shape_type < 0 ||
            object.shape_type > 2 ||
            (object.shape_type == 2 && object.footprint.empty()) ?
            0 : object.shape_type;
        object_shape.length_m = object.length_m;
        object_shape.width_m = object.width_m;
        for (const auto &point : object.footprint)
          object_shape.footprint.emplace_back(point.x, point.y);
        const auto object_polygon = behaviorSafetyWorldPolygon(
            {0.0, object.x, object.y, object.yaw_rad, 0.0},
            object_shape);
        for (const auto lane_id : route_ids) {
          const bool center_on_lane = route_map_.pointInRoadLanelet(
              lane_id, object.x, object.y);
          bool outline_on_lane = false;
          for (std::size_t index = 0;
               index < object_polygon.size() && !outline_on_lane;
               ++index) {
            const auto &a = object_polygon[index];
            const auto &b = object_polygon[
                (index + 1) % object_polygon.size()];
            const int samples = std::max(1, static_cast<int>(
                std::ceil(std::hypot(b.first - a.first,
                                     b.second - a.second) / 0.5)));
            for (int sample = 0; sample <= samples; ++sample) {
              const double ratio = static_cast<double>(sample) / samples;
              if (route_map_.pointInRoadLanelet(
                      lane_id,
                      a.first + ratio * (b.first - a.first),
                      a.second + ratio * (b.second - a.second))) {
                outline_on_lane = true;
                break;
              }
            }
          }
          if (!center_on_lane && !outline_on_lane) continue;
          observed.on_current_road_lane = true;
          observed.has_side_road_lane = center_on_lane &&
              (route_map_.hasAdjacentRoadLanelet(
                   lane_id, object_on_left) ||
               route_map_.hasOppositeRoadLanelet(
                   lane_id, object_on_left));
          break;
        }
        if (object.shape_type == 2) {
          for (const auto &point : object.footprint)
            observed.footprint.emplace_back(point.x, point.y);
        }
        observations.push_back(std::move(observed));
      }
      for (auto it = stationary_since_s_.begin();
           it != stationary_since_s_.end();) {
        if (seen.count(it->first) == 0 &&
            now_s - it->second.last_seen_s >
                compensation_s)
          it = stationary_since_s_.erase(it);
        else ++it;
      }
      std::vector<std::pair<double, double>> left_boundary;
      std::vector<std::pair<double, double>> right_boundary;
      for (const auto &point : frame.left_bound)
        left_boundary.emplace_back(point.x, point.y);
      for (const auto &point : frame.right_bound)
        right_boundary.emplace_back(point.x, point.y);
      auto ego = frame.ego;
      ego.longitudinal_velocity_mps =
          static_cast<float>(frame.ego_speed_mps);
      auto observed_inputs = makeBehaviorRouteAvoidanceObjects(
          frame.behavior_path.points, ego,
          left_boundary, right_boundary, observations, parameters);
      avoidance_perception_input_count = observed_inputs.size();
      request.avoidance->objects.insert(
          request.avoidance->objects.end(),
          std::make_move_iterator(observed_inputs.begin()),
          std::make_move_iterator(observed_inputs.end()));
    }
    if (request.lane_change &&
        request.lane_change->avoidance_activation &&
        request.lane_change->avoidance_activation->objects.empty() &&
        request.avoidance) {
      auto &activation = *request.lane_change->avoidance_activation;
      for (const auto &object : request.avoidance->objects) {
        auto unrestricted_margin = object.margin;
        unrestricted_margin.road_shoulder_distance_m = 1000.0;
        const auto geometry = staticAvoidanceObjectGeometry(
            frame.behavior_path.points, frame.ego.x, frame.ego.y,
            object.object_polygon, object.closest_reference_pose,
            object.envelope_buffer_m, object.object_on_right,
            unrestricted_margin);
        if (!geometry) continue;
        const double jerk = object.feasibility
                                .minimum_distance_jerk_mps3;
        const double minimum_distance = jerk > 0.0 ?
            4.0 * std::cbrt(
                0.5 * std::abs(
                    geometry->desired_shift_m -
                    object.feasibility.current_ego_shift_m) / jerk) *
                object.feasibility.minimum_distance_speed_mps : 0.0;
        activation.objects.push_back({
            geometry->longitudinal.start_from_ego_m,
            minimum_distance, object.classification,
            std::abs(geometry->desired_shift_m) > 0.09 &&
                staticAvoidanceShiftNecessary(
                    object.object_on_right,
                    geometry->desired_shift_m),
            object.object_on_right});
      }
    }
    if (temporary_avoidance_input) request.avoidance.reset();
    auto polygon_ids = route_ids;
    if (request.sampling) {
      for (const auto lane_id : route_ids) {
        for (const bool left : {false, true}) {
          const auto adjacent = route_map_.adjacentRoadLaneletId(
              lane_id, left);
          if (adjacent &&
              std::find(polygon_ids.begin(), polygon_ids.end(),
                        *adjacent) == polygon_ids.end())
            polygon_ids.push_back(*adjacent);
        }
      }
    }
    if (request.pull_over) {
      if (goal) request.pull_over->goal = *goal;
      if (candidate_result &&
          candidate_result->goal_safety !=
              BehaviorGoalSafetyReason::Safe)
        request.pull_over->safe = false;
      if (candidates || !cruise_perception.objects.empty()) {
        BehaviorGoalSafetyInput safety;
        safety.goal = request.pull_over->goal;
        safety.object_hard_margin_m =
            request.pull_over->object_hard_margin_m;
        const auto add_object = [&](const BehaviorSafetyPose &pose,
                                    const BehaviorSafetyObject &shape,
                                    double speed_mps) {
          auto polygon = behaviorSafetyWorldPolygon(pose, shape);
          if (speed_mps < 1.0)
            request.pull_over->path_stop_objects.push_back(polygon);
          else
            request.pull_over->path_moving_objects.push_back(polygon);
          safety.object_polygons.push_back(std::move(polygon));
        };
        if (candidates) {
          for (const auto &object : candidates->perception.objects)
            add_object(object.pose, object.shape,
                       std::hypot(object.velocity_x_mps,
                                  object.velocity_y_mps));
        } else {
          const double elapsed_s = cruise_perception.stamp_s > 0.0 ?
              std::max(0.0, frame.timestamp_s -
                                cruise_perception.stamp_s) : 0.0;
          for (const auto &object : cruise_perception.objects) {
            if (object.length_m <= 0.0 || object.width_m <= 0.0) continue;
            BehaviorSafetyObject shape;
            shape.shape_type = object.shape_type < 0 ||
                object.shape_type > 2 ||
                (object.shape_type == 2 && object.footprint.empty()) ?
                0 : object.shape_type;
            shape.length_m = object.length_m;
            shape.width_m = object.width_m;
            for (const auto &point : object.footprint)
              shape.footprint.emplace_back(point.x, point.y);
            const auto pose = currentCruiseObjectPose(
                {object.x, object.y, object.yaw_rad},
                object.predicted_paths, elapsed_s);
            add_object(
                {0.0, pose.x, pose.y, pose.yaw_rad, 0.0},
                shape,
                std::hypot(object.velocity_x_mps,
                           object.velocity_y_mps));
          }
        }
        if (evaluateBehaviorGoalSafety(safety) !=
            BehaviorGoalSafetyReason::Safe)
          request.pull_over->safe = false;
      }
      for (const auto lane_id :
           request.pull_over->goal_lanelet_ids) {
        if (std::find(polygon_ids.begin(), polygon_ids.end(), lane_id) ==
            polygon_ids.end()) polygon_ids.push_back(lane_id);
      }
    }
    if (request.sampling &&
        request.sampling->obstacle_polygons.empty()) {
      const double elapsed_s = cruise_perception.stamp_s > 0.0 ?
          std::max(0.0, frame.timestamp_s -
                            cruise_perception.stamp_s) : 0.0;
      for (const auto &object : cruise_perception.objects) {
        if (object.length_m <= 0.0 || object.width_m <= 0.0)
          continue;
        BehaviorSafetyObject shape;
        shape.shape_type = object.shape_type < 0 ||
            object.shape_type > 2 ||
            (object.shape_type == 2 && object.footprint.empty()) ?
            0 : object.shape_type;
        shape.length_m = object.length_m;
        shape.width_m = object.width_m;
        for (const auto &point : object.footprint)
          shape.footprint.emplace_back(point.x, point.y);
        const auto pose = currentCruiseObjectPose(
            {object.x, object.y, object.yaw_rad},
            object.predicted_paths, elapsed_s);
        request.sampling->obstacle_polygons.push_back(
            behaviorSafetyWorldPolygon(
                {0.0, pose.x, pose.y, pose.yaw_rad, 0.0},
                shape));
      }
    }
    if (request.lane_change &&
        request.lane_change->avoidance_activation &&
        request.lane_change->external_direction ==
            BehaviorRouteLaneChangeRequest::ExternalDirection::None) {
      const auto &objects =
          request.lane_change->avoidance_activation->objects;
      const auto nearest = std::min_element(
          objects.begin(), objects.end(), [](const auto &a, const auto &b) {
            return a.longitudinal_distance_m < b.longitudinal_distance_m;
          });
      if (nearest != objects.end()) {
        request.lane_change->external_direction = nearest->object_on_right ?
            BehaviorRouteLaneChangeRequest::ExternalDirection::Left :
            BehaviorRouteLaneChangeRequest::ExternalDirection::Right;
      }
    }
    if (request.lane_change &&
        request.lane_change->external_direction !=
            BehaviorRouteLaneChangeRequest::ExternalDirection::None) {
      std::int64_t current_lane_id = route_ids.front();
      for (const auto lane_id : route_ids) {
        if (route_map_.pointInRoadLanelet(
                lane_id, frame.ego.x, frame.ego.y)) {
          current_lane_id = lane_id;
          break;
        }
      }
      const bool left = request.lane_change->external_direction ==
          BehaviorRouteLaneChangeRequest::ExternalDirection::Left;
      const auto adjacent = route_map_.adjacentRoadLaneletId(
          current_lane_id, left);
      if (adjacent)
        request.lane_change->target_lanelet_ids = {*adjacent};
      else
        request.lane_change.reset();
    }
    if (request.lane_change &&
        !request.lane_change->target_lanelet_ids.empty()) {
      if (!request.lane_change->regulation.stopped_time_s)
        request.lane_change->regulation.stopped_time_s =
            ego_stopped_since_s_ ?
                std::max(0.0, frame.timestamp_s -
                                  *ego_stopped_since_s_) : 0.0;
      request.lane_change->regulation.distances =
          route_map_.laneChangeRegulatoryDistances(
              frame.behavior_path.points, route_ids,
              frame.ego.x, frame.ego.y);
      const auto ego_index = nearestIndex(frame.behavior_path.points,
                                          frame.ego);
      std::size_t change_index = ego_index;
      double traveled = 0.0;
      const double prepare_distance =
          request.lane_change->sampling.enabled ? 0.0 :
          request.lane_change->prepare_length_m;
      while (change_index + 1 < frame.behavior_path.points.size() &&
             traveled < prepare_distance) {
        const auto &a = frame.behavior_path.points[change_index];
        const auto &b = frame.behavior_path.points[change_index + 1];
        traveled += std::hypot(b.x - a.x, b.y - a.y);
        ++change_index;
      }
      const auto &change_point = frame.behavior_path.points[change_index];
      const auto target = route_map_.laneChangeTargetReference(
          request.lane_change->target_lanelet_ids,
          {change_point.x, change_point.y},
          request.lane_change->shift_length_m, effective_route_speed_mps,
          std::max(30.0, request.lane_change->shift_length_m + 10.0),
          0.0, goal ? std::optional<BehaviorPathShiftAnchor>{
              {goal->x, goal->y}} : std::nullopt);
      if (target) {
        request.lane_change->target_reference = target->path;
        request.lane_change->target_reference_lane_ids = target->lane_ids;
      }
      if (request.lane_change->avoidance_activation &&
          request.lane_change->avoidance_activation
              ->minimum_lane_change_buffer_m <= 0.0) {
        auto &activation =
            *request.lane_change->avoidance_activation;
        const double lateral_shift = target ?
            std::abs(lateralOffset(target->path, frame.ego)) :
            std::numeric_limits<double>::infinity();
        activation.minimum_lane_change_buffer_m =
            behaviorMinimumLaneChangeDistanceBuffer(
                lateral_shift,
                request.lane_change->sampling
                    .minimum_change_velocity_mps,
                request.lane_change->sampling.lateral_jerk_mps3,
                request.lane_change->sampling
                    .maximum_lateral_acceleration_mps2,
                activation.finish_judge_buffer_m);
      }
      for (const auto lane_id : request.lane_change->target_lanelet_ids) {
        if (std::find(polygon_ids.begin(), polygon_ids.end(), lane_id) ==
            polygon_ids.end()) polygon_ids.push_back(lane_id);
      }
      if (request.lane_change->use_perception_objects) {
        // generateExpandedLanelets expands only the outer side of the target
        // lane in the lane-change direction. The launch config uses 1 m for
        // either direction.
        const auto target_centerline = route_map_.laneCenterline(
            request.lane_change->target_lanelet_ids.front());
        TrajectoryPoint target_point;
        if (target && !target->path.empty()) {
          target_point = target->path.front();
        } else if (!target_centerline.empty()) {
          target_point.x = target_centerline.front().x;
          target_point.y = target_centerline.front().y;
        }
        const bool target_to_left =
            request.lane_change->external_direction ==
                BehaviorRouteLaneChangeRequest::ExternalDirection::Left ||
            (request.lane_change->external_direction ==
                 BehaviorRouteLaneChangeRequest::ExternalDirection::None &&
             lateralOffset(frame.behavior_path.points, target_point) > 0.0);
        const auto expanded_target = expandedLanePolygon(
            route_map_, request.lane_change->target_lanelet_ids.front(),
            target_to_left ? 1.0 : 0.0,
            target_to_left ? 0.0 : 1.0);
        request.lane_change->safety.objects =
            laneChangePerceptionObjects(
                frame.behavior_path.points, frame.ego,
                cruise_perception, frame.timestamp_s,
                route_map_.lanePolygons(route_ids),
                route_map_.lanePolygons(
                    request.lane_change->target_lanelet_ids),
                expanded_target);
        if (request.lane_change->safety.safety.check_lane_regions) {
          const auto current_polygons =
              route_map_.lanePolygons(route_ids);
          if (!current_polygons.empty())
            request.lane_change->safety.safety.current_lane_region =
                safetyPolygon(current_polygons.front());
          request.lane_change->safety.safety
              .expanded_target_lane_region = expanded_target;
        }
      }
    }
    std::vector<std::int64_t> lane_change_route_signature;
    for (const auto &section : route.sections) {
      lane_change_route_signature.push_back(section.preferred_lanelet_id);
      lane_change_route_signature.insert(
          lane_change_route_signature.end(),
          section.primitive_lanelet_ids.begin(),
          section.primitive_lanelet_ids.end());
      lane_change_route_signature.push_back(0);
    }
    action_result = composeBehaviorRouteAction(
        frame.behavior_path, frame.ego,
        route_map_.lanePolygons(polygon_ids), request,
        registered_avoidance_);
    if (action_result->kind == BehaviorRouteActionKind::Sampling) {
      active_sampling_path_ = action_result->path.points;
      for (std::size_t index = 0;
           index < action_result->path.points.size(); ++index) {
        const auto &point = action_result->path.points[index];
        std::vector<std::int64_t> containing;
        for (const auto lane_id : polygon_ids) {
          if (route_map_.pointInRoadLanelet(
                  lane_id, point.x, point.y))
            containing.push_back(lane_id);
        }
        if (!containing.empty())
          action_result->path.lane_ids[index] =
              std::move(containing);
        else if (index > 0)
          action_result->path.lane_ids[index] =
              action_result->path.lane_ids[index - 1];
      }
    }
    const bool side_shift_finished = request.side_shift &&
        std::abs(request.side_shift->requested_lateral_offset_m) < 1.0e-4 &&
        std::abs(request.side_shift->current_lateral_offset_m) < 0.01;
    const bool may_keep_side_shift = request.side_shift &&
        request.side_shift->approved &&
        !side_shift_finished &&
        (action_result->kind == BehaviorRouteActionKind::LaneFollow ||
         action_result->kind == BehaviorRouteActionKind::SideShift);
    const bool reuse_side_shift = may_keep_side_shift &&
        active_side_shift_action_ && active_side_shift_request_ &&
        request.side_shift->requested_lateral_offset_m ==
            active_side_shift_request_->requested_lateral_offset_m &&
        request.side_shift->parameters.minimum_start_distance_m ==
            active_side_shift_request_->parameters.minimum_start_distance_m &&
        request.side_shift->parameters.start_delay_s ==
            active_side_shift_request_->parameters.start_delay_s &&
        request.side_shift->parameters.lateral_jerk_mps3 ==
            active_side_shift_request_->parameters.lateral_jerk_mps3 &&
        request.side_shift->parameters.minimum_shift_distance_m ==
            active_side_shift_request_->parameters.minimum_shift_distance_m &&
        request.side_shift->parameters.minimum_shift_speed_mps ==
            active_side_shift_request_->parameters.minimum_shift_speed_mps &&
        request.side_shift->parameters.request_time_limit_s ==
            active_side_shift_request_->parameters.request_time_limit_s;
    const bool defer_side_shift_change = may_keep_side_shift &&
        active_side_shift_action_ && active_side_shift_request_ &&
        request.side_shift->requested_lateral_offset_m !=
            active_side_shift_request_->requested_lateral_offset_m &&
        ((active_side_shift_action_->side_shift_end_index &&
          nearestIndex(active_side_shift_action_->path.points, frame.ego) <
              *active_side_shift_action_->side_shift_end_index) ||
         (last_side_shift_request_time_s_ &&
          frame.timestamp_s >= *last_side_shift_request_time_s_ &&
          frame.timestamp_s - *last_side_shift_request_time_s_ <
              request.side_shift->parameters.request_time_limit_s));
    if (reuse_side_shift || defer_side_shift_change) {
      action_result = *active_side_shift_action_;
    } else {
      active_side_shift_action_.reset();
      active_side_shift_request_.reset();
      if (may_keep_side_shift &&
          action_result->kind == BehaviorRouteActionKind::SideShift) {
        active_side_shift_action_ = *action_result;
        active_side_shift_request_ = *request.side_shift;
        last_side_shift_request_time_s_ = frame.timestamp_s;
      }
    }
    const bool may_keep_start = request.start &&
        (request.start->use_geometric ||
         request.start->use_geometric_fallback ||
         request.start->use_freespace ||
         request.start->use_freespace_fallback ||
         active_start_action_.has_value() ||
         action_result->start_phase_end_indices.size() > 1) &&
        (action_result->kind == BehaviorRouteActionKind::LaneFollow ||
         action_result->kind == BehaviorRouteActionKind::StartPullOut);
    const bool reuse_start = may_keep_start &&
        active_start_action_ && active_start_request_ &&
        sameStartPlan(*request.start, *active_start_request_) &&
        !behaviorPullOverPathCollides(
            active_start_action_->path.points, frame.ego,
            request.start->vehicle,
            request.start->path_stop_objects,
            request.start->object_hard_margin_m);
    if (reuse_start) {
      action_result = *active_start_action_;
    } else {
      active_start_action_.reset();
      active_start_request_.reset();
      start_phase_ = {};
      if (may_keep_start &&
          action_result->kind == BehaviorRouteActionKind::StartPullOut &&
          action_result->start_phase_end_indices.size() > 1) {
        active_start_action_ = *action_result;
        active_start_request_ = *request.start;
      }
    }
    const bool may_keep_pull_over = request.pull_over &&
        request.pull_over->approved && request.pull_over->safe &&
        (action_result->kind == BehaviorRouteActionKind::LaneFollow ||
         action_result->kind == BehaviorRouteActionKind::PullOver);
    const bool reuse_pull_over = may_keep_pull_over &&
        active_pull_over_action_ && active_pull_over_request_ &&
        samePullOverPlan(*request.pull_over,
                         *active_pull_over_request_) &&
        !behaviorPullOverPathCollides(
            active_pull_over_action_->path.points, frame.ego,
            request.pull_over->vehicle,
            request.pull_over->path_stop_objects,
            request.pull_over->object_hard_margin_m,
            request.pull_over->maximum_deceleration_mps2,
            request.pull_over->maximum_extra_stopping_margin_m) &&
        !behaviorPullOverPathCollides(
            active_pull_over_action_->path.points, frame.ego,
            request.pull_over->vehicle,
            request.pull_over->path_moving_objects,
            request.pull_over->object_hard_margin_m,
            request.pull_over->maximum_deceleration_mps2,
            request.pull_over->maximum_extra_stopping_margin_m);
    if (reuse_pull_over) {
      action_result = *active_pull_over_action_;
    } else {
      active_pull_over_action_.reset();
      active_pull_over_request_.reset();
      pull_over_phase_ = {};
      if (may_keep_pull_over &&
          action_result->kind == BehaviorRouteActionKind::PullOver &&
          action_result->pull_over_phase_end_indices.size() > 1) {
        active_pull_over_action_ = *action_result;
        active_pull_over_request_ = *request.pull_over;
      }
    }
    // LaneChangeInterface::updateData only refreshes the selected candidate
    // while waiting for approval (or aborting). A running module keeps the
    // approved shift path until hasFinishedLaneChange or a failure transition.
    // Re-searching from each new ego pose moves the shift start and loses that
    // execution state.
    bool lane_change_finished = false;
    if (active_lane_change_action_ &&
        active_lane_change_action_->lane_change_shift_end_index &&
        !active_lane_change_action_->path.points.empty()) {
      const auto &path = active_lane_change_action_->path.points;
      const auto end_index = std::min(
          *active_lane_change_action_->lane_change_shift_end_index,
          path.size() - 1);
      const auto &end = path[end_index];
      const double remaining = longitudinalArc(path, end.x, end.y) -
          longitudinalArc(path, frame.ego.x, frame.ego.y);
      const double finish_buffer = std::abs(frame.ego_speed_mps) < 1.0 ?
          0.0 : 2.0;
      const bool on_target = !active_lane_change_target_ids_.empty() &&
          route_map_.pointInRoadLanelet(
              active_lane_change_target_ids_.front(),
              frame.ego.x, frame.ego.y);
      lane_change_finished = on_target && remaining + finish_buffer < 0.0;
      if (!lane_change_finished &&
          active_lane_change_target_reference_.size() >= 2) {
        const auto &target = active_lane_change_target_reference_;
        const auto index = nearestIndex(target, frame.ego);
        const auto &a = target[std::min(index, target.size() - 2)];
        const auto &b = target[std::min(index + 1, target.size() - 1)];
        const double yaw_error = std::abs(std::remainder(
            pointYaw(frame.ego) - std::atan2(b.y - a.y, b.x - a.x),
            2.0 * std::acos(-1.0)));
        lane_change_finished = on_target &&
            std::abs(lateralOffset(target, frame.ego)) < 0.1 &&
            yaw_error < std::acos(-1.0) / 180.0;
      }
    }
    // A RUNNING source module keeps its approved path even when a later
    // candidate search returns no request. Explicit deactivation, a different
    // target, route change, or completion still releases it.
    const bool execution_active = request.lane_change ?
        request.lane_change->execution.activated :
        route_lane_change_activated;
    const bool same_lane_change = execution_active &&
        (!request.lane_change ||
         (request.lane_change->target_lanelet_ids ==
              active_lane_change_target_ids_ &&
          (!request.lane_change->avoidance_activation ||
           behaviorAvoidanceByLaneChangeRequested(
               *request.lane_change->avoidance_activation)) &&
          !request.lane_change->execution.failure.force_deactivated)) &&
        lane_change_route_signature == active_lane_change_route_signature_;
    const bool keep_lane_change = active_lane_change_action_ &&
        same_lane_change && !lane_change_finished &&
        (action_result->kind == BehaviorRouteActionKind::LaneFollow ||
         action_result->kind == BehaviorRouteActionKind::LaneChange);
    if (keep_lane_change) {
      action_result = *active_lane_change_action_;
    } else {
      active_lane_change_action_.reset();
      active_lane_change_target_ids_.clear();
      active_lane_change_route_signature_.clear();
      active_lane_change_target_reference_.clear();
      if (request.lane_change &&
          action_result->kind == BehaviorRouteActionKind::LaneChange &&
          action_result->lane_change_shift_end_index) {
        active_lane_change_action_ = *action_result;
        active_lane_change_target_ids_ =
            request.lane_change->target_lanelet_ids;
        active_lane_change_route_signature_ =
            std::move(lane_change_route_signature);
        active_lane_change_target_reference_ =
            request.lane_change->target_reference;
      }
    }
    const auto *running_target_ids = request.lane_change ?
        &request.lane_change->target_lanelet_ids :
        &active_lane_change_target_ids_;
    if (action_result->kind == BehaviorRouteActionKind::LaneChange &&
        !running_target_ids->empty()) {
      const auto target_id = running_target_ids->front();
      const auto centerline = route_map_.laneCenterline(target_id);
      const auto bounds = route_map_.laneBounds(target_id);
      if (!centerline.empty() && bounds.left.size() >= 2 &&
          bounds.right.size() >= 2) {
        TrajectoryPoint target_point;
        target_point.x = centerline.front().x;
        target_point.y = centerline.front().y;
        const bool to_left = lateralOffset(
            frame.behavior_path.points, target_point) > 0.0;
        auto &outer = to_left ? frame.left_bound : frame.right_bound;
        outer.clear();
        const auto &source = to_left ? bounds.left : bounds.right;
        for (const auto &point : source)
          outer.push_back({point.x, point.y});
      }
    }
    if (action_result->kind == BehaviorRouteActionKind::StartPullOut &&
        action_result->start_phase_end_indices.size() > 1 &&
        request.start) {
      const auto &end = action_result->path.points.back();
      action_result->path = selectBehaviorStartPhase(
          action_result->path,
          action_result->start_phase_end_indices,
          frame.ego, frame.ego_speed_mps, frame.timestamp_s,
          {end.x, end.y, 0.0}, start_phase_,
          request.start->terminal_is_goal);
      action_result->start_current_phase = start_phase_.index;
    } else {
      start_phase_ = {};
    }
    if (action_result->kind == BehaviorRouteActionKind::PullOver) {
      if (request.pull_over &&
          action_result->pull_over_phase_end_indices.size() > 1) {
        action_result->path = selectBehaviorPullOverPhase(
            action_result->path,
            action_result->pull_over_phase_end_indices,
            frame.ego, frame.ego_speed_mps, frame.timestamp_s,
            request.pull_over->goal, pull_over_phase_);
        action_result->pull_over_current_phase = pull_over_phase_.index;
      } else {
        pull_over_phase_ = {};
      }
      frame.goal_contained = true;
      if (request.pull_over &&
          !request.pull_over->goal_lanelet_ids.empty()) {
        const auto lane_id = request.pull_over->goal_lanelet_ids.front();
        const auto bounds = route_map_.laneBounds(lane_id);
        if (bounds.left.size() >= 2 && bounds.right.size() >= 2) {
          TrajectoryPoint goal_point;
          goal_point.x = request.pull_over->goal.x;
          goal_point.y = request.pull_over->goal.y;
          const bool to_left = lateralOffset(
              frame.behavior_path.points, goal_point) > 0.0;
          auto &outer = to_left ? frame.left_bound : frame.right_bound;
          outer.clear();
          const auto &source = to_left ? bounds.left : bounds.right;
          for (const auto &point : source)
            outer.push_back({point.x, point.y});
        }
      }
    } else {
      pull_over_phase_ = {};
    }
    if (action_result->kind == BehaviorRouteActionKind::SideShift &&
        request.side_shift) {
      const auto &applied = active_side_shift_request_ ?
          *active_side_shift_request_ : *request.side_shift;
      const double minimum_shift = std::min(
          applied.current_lateral_offset_m,
          applied.requested_lateral_offset_m);
      const double maximum_shift = std::max(
          applied.current_lateral_offset_m,
          applied.requested_lateral_offset_m);
      const double left_expansion = std::max(
          0.0, maximum_shift +
              (minimum_shift > 0.1 ? 0.5 : 0.0));
      const double right_expansion = std::max(
          0.0, -minimum_shift +
              (maximum_shift < -0.1 ? 0.5 : 0.0));
      expandSideShiftBoundary(frame.left_bound, left_expansion);
      expandSideShiftBoundary(frame.right_bound, -right_expansion);
    }
    frame.behavior_path = action_result->path;
    registered_avoidance_ = action_result->registered_shift_arcs;
  } else {
    pull_over_phase_ = {};
    start_phase_ = {};
    active_start_action_.reset();
    active_start_request_.reset();
    active_side_shift_action_.reset();
    active_side_shift_request_.reset();
    last_side_shift_request_time_s_.reset();
  }
  if (nominal_route &&
      (!action_result ||
       action_result->kind != BehaviorRouteActionKind::LaneChange)) {
    frame.behavior_path = resampleRouteReference(
        frame.behavior_path, true, goal.has_value());
    frame.behavior_path = cropRouteReference(
        frame.behavior_path, frame.ego, route_backward_path_length_m_);
  }
  const auto dynamic_drivable_area = extractDynamicDrivableArea(
      frame.behavior_path.points, frame.left_bound, frame.right_bound,
      dynamic_avoidance_candidate_polygons,
      dynamic_avoidance_collision_on_left);
  frame.left_bound = dynamic_drivable_area.left_bound;
  frame.right_bound = dynamic_drivable_area.right_bound;
  frame.behavior_path.left_bound.clear();
  frame.behavior_path.right_bound.clear();
  for (const auto &point : frame.left_bound)
    frame.behavior_path.left_bound.push_back({point.x, point.y, 0.0});
  for (const auto &point : frame.right_bound)
    frame.behavior_path.right_bound.push_back({point.x, point.y, 0.0});
  auto route_velocity_context = velocity_context;
  if (nominal_route) route_velocity_context.blind_spot_auto_enabled = true;
  auto result = onFrame(
      frame, route_velocity_context, cruise_perception, pointcloud,
      cruise_callback_perception, out_of_lane_input);
  result.behavior_candidates = std::move(candidate_result);
  result.behavior_action = std::move(action_result);
  result.avoidance_perception_input_count =
      avoidance_perception_input_count;
  result.dynamic_avoidance_targets =
      std::move(dynamic_avoidance_targets);
  result.dynamic_avoidance_final_targets =
      std::move(dynamic_avoidance_final_targets);
  result.dynamic_avoidance_selected_targets =
      std::move(dynamic_avoidance_selected_targets);
  result.dynamic_avoidance_candidate_polygons =
      std::move(dynamic_avoidance_candidate_polygons);
  result.dynamic_drivable_area = dynamic_drivable_area;
  result.generated_parking_grid = std::move(generated_parking_grid);
  result.route_reference_path = std::move(route_reference_path);
  result.route_goal_refined_path = std::move(route_goal_refined_path);
  result.route_behavior_path = std::move(frame.behavior_path);
  return result;
}

MotionVelocityPlan PlanningPipelineMapAdapter::planOutOfLane(
    const Trajectory &path, const TrajectoryPoint &ego,
    double now_s, const PlanningOutOfLaneInput &input) {
  if (path.size() < 2 ||
      (!input.time_from_start_s.empty() &&
       input.time_from_start_s.size() != path.size()) ||
      (!input.minimum_stop_arc_m && !input.braking) ||
      (input.minimum_stop_arc_m &&
       (!std::isfinite(*input.minimum_stop_arc_m) ||
        *input.minimum_stop_arc_m < 0.0)) ||
      !std::isfinite(input.stop_line_search_distance_m) ||
      input.stop_line_search_distance_m < 0.0)
    throw std::invalid_argument(
        "out-of-lane callback requires path-matched arrival times and braking arc");
  const Trajectory sampled = input.time_from_start_s.empty() ?
      resampleMotionVelocityPlanningInput(path) : path;
  const auto times = input.time_from_start_s.empty() ?
      motionArrivalTimes(sampled, ego) : input.time_from_start_s;
  const MotionVelocityPoint2D ego_position{ego.x, ego.y};
  const double minimum_stop_arc = input.minimum_stop_arc_m ?
      *input.minimum_stop_arc_m : outOfLaneMinimumStopArc(
          sampled, ego, *input.braking,
          out_of_lane_.previousSlowdownPoint());
  const auto candidate = route_map_.findOutOfLaneMapCandidateFromTrackedObjects(
      sampled, times, ego_position, input.objects,
      input.red_traffic_light_ids, input.stop_line_search_distance_m,
      minimum_stop_arc);
  return out_of_lane_.plan(sampled, ego_position, candidate, now_s);
}

bool PlanningPipelineMapAdapter::loadBehaviorVelocityModule(
    BehaviorVelocityModule module) {
  if (!map_planner_->loadModule(module)) return false;
  modules_.push_back(module);
  return true;
}

bool PlanningPipelineMapAdapter::unloadBehaviorVelocityModule(
    BehaviorVelocityModule module) {
  if (!map_planner_->unloadModule(module)) return false;
  const auto found = std::find(modules_.begin(), modules_.end(), module);
  if (found != modules_.end()) modules_.erase(found);
  return true;
}

bool PlanningPipelineMapAdapter::loadBehaviorVelocityPlugin(
    std::string_view class_name) {
  if (class_name ==
      "autoware::behavior_velocity_planner::IntersectionModulePlugin") {
    const auto has = [&](BehaviorVelocityModule module) {
      return std::find(modules_.begin(), modules_.end(), module) !=
          modules_.end();
    };
    if (has(BehaviorVelocityModule::IntersectionStuck) ||
        has(BehaviorVelocityModule::IntersectionCollision)) return false;
    if (!loadBehaviorVelocityModule(BehaviorVelocityModule::IntersectionStuck))
      return false;
    if (!loadBehaviorVelocityModule(
            BehaviorVelocityModule::IntersectionCollision)) {
      unloadBehaviorVelocityModule(BehaviorVelocityModule::IntersectionStuck);
      return false;
    }
    return true;
  }
  const auto module = behaviorVelocityLoadClass(class_name);
  return module && loadBehaviorVelocityModule(*module);
}

bool PlanningPipelineMapAdapter::unloadBehaviorVelocityPlugin(
    std::string_view module_name) {
  if (module_name == "intersection") {
    const auto has = [&](BehaviorVelocityModule module) {
      return std::find(modules_.begin(), modules_.end(), module) !=
          modules_.end();
    };
    if (!has(BehaviorVelocityModule::IntersectionStuck) ||
        !has(BehaviorVelocityModule::IntersectionCollision)) return false;
    const bool removed_collision = unloadBehaviorVelocityModule(
        BehaviorVelocityModule::IntersectionCollision);
    const bool removed_stuck = unloadBehaviorVelocityModule(
        BehaviorVelocityModule::IntersectionStuck);
    return removed_collision && removed_stuck;
  }
  const auto module = behaviorVelocityUnloadName(module_name);
  return module && unloadBehaviorVelocityModule(*module);
}

bool PlanningPipelineMapAdapter::loadMotionVelocityPlugin(
    std::string_view class_name) {
  const auto module = motionPluginClass(class_name);
  if (!module || std::find(motion_modules_.begin(), motion_modules_.end(),
                           *module) != motion_modules_.end()) return false;
  motion_modules_.push_back(*module);
  return true;
}

bool PlanningPipelineMapAdapter::unloadMotionVelocityPlugin(
    std::string_view class_name) {
  const auto module = motionPluginClass(class_name);
  if (!module) return false;
  const auto found = std::find(motion_modules_.begin(), motion_modules_.end(),
                               *module);
  if (found == motion_modules_.end()) return false;
  motion_modules_.erase(found);
  if (*module == MotionVelocityModule::OutOfLane) out_of_lane_.reset();
  if (*module == MotionVelocityModule::DynamicObstacleStop) dynamic_stop_.reset();
  return true;
}

void PlanningPipelineMapAdapter::reset() {
  map_planner_ = std::make_unique<BehaviorVelocityMapPlanner>(
      map_data_, modules_);
  pipeline_.reset();
  dynamic_stop_.reset();
  out_of_lane_.reset();
  out_of_lane_input_.reset();
  cruise_candidates_.reset();
  cruise_pid_.reset();
  velocity_limit_selector_ = VelocityLimitSelector(selector_config_);
  cruise_limit_active_ = false;
  cruise_slowdown_.reset();
  cruise_slowdown_candidates_.reset();
  pointcloud_obstacles_.clear();
  last_slowdown_candidate_count_ = 0;
  last_slowdown_intervals_.clear();
  last_velocity_result_.reset();
  active_route_ids_.clear();
  route_signature_.clear();
  current_route_lane_id_.reset();
  registered_avoidance_.clear();
  pull_over_phase_ = {};
  active_pull_over_action_.reset();
  active_pull_over_request_.reset();
  start_phase_ = {};
  active_start_action_.reset();
  active_start_request_.reset();
  active_side_shift_action_.reset();
  active_side_shift_request_.reset();
  active_lane_change_action_.reset();
  active_lane_change_target_ids_.clear();
  active_lane_change_route_signature_.clear();
  active_lane_change_target_reference_.clear();
  active_sampling_path_.clear();
  dynamic_avoidance_tracker_.reset();
  dynamic_avoidance_parameters_.reset();
  dynamic_avoidance_last_objects_.clear();
  dynamic_avoidance_feasibility_origin_.clear();
  dynamic_avoidance_previous_boundary_.clear();
  dynamic_inside_last_time_s_.clear();
  last_side_shift_request_time_s_.reset();
  ego_stopped_since_s_.reset();
  stationary_since_s_.clear();
}

}  // planning 네임스페이스
