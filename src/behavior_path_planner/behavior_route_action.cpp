// SPDX-License-Identifier: Apache-2.0
// 출발·회피·차로 변경·주차 후보를 검사해 행동 경로로 선택합니다.

#include "planning/behavior_route_action.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace planning {
namespace {

std::size_t nearestIndex(const Trajectory &path, double x, double y) {
  std::size_t nearest = 0;
  double distance = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0; index < path.size(); ++index) {
    const double candidate = std::hypot(path[index].x - x, path[index].y - y);
    if (candidate < distance) {
      distance = candidate;
      nearest = index;
    }
  }
  return nearest;
}

BehaviorGoalPath withGeometry(const BehaviorGoalPath &reference,
                              Trajectory path) {
  BehaviorGoalPath result = reference;
  result.points = std::move(path);
  result.lane_ids.clear();
  result.lane_ids.reserve(result.points.size());
  for (std::size_t index = 0; index < result.points.size(); ++index) {
    const auto source = result.points.size() == reference.points.size() ?
        index : nearestIndex(reference.points, result.points[index].x,
                             result.points[index].y);
    result.lane_ids.push_back(reference.lane_ids[source]);
  }
  return result;
}

double signedLateral(const TrajectoryPoint &path_point,
                     double x, double y) {
  const double yaw = std::atan2(
      2.0 * (path_point.orientation_w * path_point.orientation_z +
             path_point.orientation_x * path_point.orientation_y),
      1.0 - 2.0 * (path_point.orientation_y * path_point.orientation_y +
                   path_point.orientation_z * path_point.orientation_z));
  return -(x - path_point.x) * std::sin(yaw) +
         (y - path_point.y) * std::cos(yaw);
}

double poseYaw(const TrajectoryPoint &point) {
  return std::atan2(
      2.0 * (point.orientation_w * point.orientation_z +
             point.orientation_x * point.orientation_y),
      1.0 - 2.0 * (point.orientation_y * point.orientation_y +
                   point.orientation_z * point.orientation_z));
}

bool pullOverPathCollides(
    const Trajectory &path,
    const TrajectoryPoint &ego,
    const BehaviorPathVehicleFootprint &vehicle,
    const std::vector<BehaviorSafetyPolygon> &objects,
    double margin_m,
    double maximum_deceleration_mps2 = 1.0,
    double maximum_extra_stopping_margin_m = 0.0) {
  if (objects.empty()) return false;
  if (path.empty() || !std::isfinite(maximum_deceleration_mps2) ||
      maximum_deceleration_mps2 <= 0.0 ||
      !std::isfinite(maximum_extra_stopping_margin_m) ||
      maximum_extra_stopping_margin_m < 0.0) return true;
  const double spacing = std::max(0.1, vehicle.sample_spacing_m);
  const std::size_t start_index = nearestIndex(
      path, ego.x, ego.y);
  for (std::size_t index = start_index; index < path.size(); ++index) {
    const auto &a = path[index];
    const auto &b = path[std::min(index + 1, path.size() - 1)];
    const double distance = std::hypot(b.x - a.x, b.y - a.y);
    const int samples = std::max(1, static_cast<int>(
        std::ceil(distance / spacing)));
    const double first_yaw = poseYaw(a);
    const double yaw_difference = std::remainder(
        poseYaw(b) - first_yaw, 2.0 * std::acos(-1.0));
    for (int sample = 0; sample < samples; ++sample) {
      const double ratio = static_cast<double>(sample) / samples;
      const double velocity = std::abs(
          (1.0 - ratio) * a.longitudinal_velocity_mps +
          ratio * b.longitudinal_velocity_mps);
      const double stopping_margin = std::min(
          maximum_extra_stopping_margin_m,
          velocity * velocity /
              (2.0 * maximum_deceleration_mps2));
      const double curvature = distance > 1.0e-6 ?
          std::abs(yaw_difference / distance) : 0.0;
      const double lateral_margin = std::min(
          stopping_margin, curvature * velocity * velocity);
      const BehaviorSafetyFootprint footprint{
          vehicle.front_m + stopping_margin,
          vehicle.back_m,
          vehicle.half_width_m + lateral_margin};
      const BehaviorSafetyPose pose{
          0.0, a.x + ratio * (b.x - a.x),
          a.y + ratio * (b.y - a.y),
          first_yaw + ratio * yaw_difference, 0.0};
      if (behaviorFootprintCollidesWithObjects(
              pose, footprint, objects,
              std::max(margin_m, 1.0e-9))) return true;
    }
  }
  return false;
}

bool referenceCurvesNearGoal(const Trajectory &path,
                             const BehaviorGoalPose &goal) {
  if (path.size() < 5) return false;
  const auto center = nearestIndex(path, goal.x, goal.y);
  const auto before = center > 8 ? center - 8 : 0;
  const auto after = std::min(center + 8, path.size() - 1);
  if (before == center || after == center) return false;
  const double first_yaw = std::atan2(
      path[center].y - path[before].y,
      path[center].x - path[before].x);
  const double second_yaw = std::atan2(
      path[after].y - path[center].y,
      path[after].x - path[center].x);
  return std::abs(std::remainder(
      second_yaw - first_yaw, 2.0 * std::acos(-1.0))) > 0.03;
}

double boundaryDistance(
    const std::vector<std::pair<double, double>> &boundary,
    double x, double y) {
  double closest = std::numeric_limits<double>::infinity();
  for (std::size_t index = 1; index < boundary.size(); ++index) {
    const auto &[ax, ay] = boundary[index - 1];
    const auto &[bx, by] = boundary[index];
    const double dx = bx - ax;
    const double dy = by - ay;
    const double length_squared = dx * dx + dy * dy;
    if (length_squared < 1.0e-12) continue;
    const double fraction = std::clamp(
        ((x - ax) * dx + (y - ay) * dy) / length_squared,
        0.0, 1.0);
    closest = std::min(closest, std::hypot(
        x - ax - fraction * dx, y - ay - fraction * dy));
  }
  return closest;
}

double distanceForShift(double shift_m, double jerk_mps3,
                        double speed_mps) {
  return 4.0 * std::cbrt(
      0.5 * std::abs(shift_m) / jerk_mps3) * speed_mps;
}

std::optional<double> regulatoryLimit(
    const BehaviorRouteLaneChangeRequest &request) {
  const auto &regulation = request.regulation;
  if (regulation.near_current_terminal_start) return std::nullopt;
  const bool stopped = regulation.stopped_time_s.value_or(0.0) >=
      regulation.stop_time_threshold_s;
  return laneChangeNextRegulatoryDistance(
      regulation.distances, regulation.intersection && !stopped,
      regulation.crosswalk && !stopped, regulation.traffic_light);
}

bool blockedBeforeRegulation(
    const BehaviorRouteLaneChangeRequest &request) {
  const auto distance = regulatoryLimit(request);
  if (!distance) return false;
  const double maximum_prepare = request.sampling.enabled ?
      request.sampling.maximum_prepare_duration_s *
          request.sampling.maximum_vehicle_velocity_mps :
      request.prepare_length_m;
  return std::isfinite(maximum_prepare) &&
      maximum_prepare > *distance;
}

std::optional<BehaviorLaneChangeCandidateSearchResult> searchRouteLaneChange(
    const BehaviorGoalPath &reference, const TrajectoryPoint &ego,
    const BehaviorRouteLaneChangeRequest &request,
    std::size_t &prepare_point_count) {
  if (request.target_reference.size() < 3 ||
      !std::isfinite(request.prepare_length_m) ||
      !std::isfinite(request.shift_length_m) ||
      request.prepare_length_m <= 0.0 || request.shift_length_m <= 0.0 ||
      !std::isfinite(request.lateral_acceleration_mps2) ||
      request.lateral_acceleration_mps2 <= 0.0) return std::nullopt;
  if (blockedBeforeRegulation(request)) return std::nullopt;
  const auto start_index = nearestIndex(reference.points, ego.x, ego.y);
  if (start_index + 2 >= reference.points.size()) return std::nullopt;
  std::size_t end_index = start_index;
  double prepare_length = 0.0;
  while (end_index + 1 < reference.points.size() &&
         prepare_length < request.prepare_length_m) {
    prepare_length += std::hypot(
        reference.points[end_index + 1].x - reference.points[end_index].x,
        reference.points[end_index + 1].y - reference.points[end_index].y);
    ++end_index;
  }
  if (end_index <= start_index ||
      prepare_length + 0.5 < request.prepare_length_m) return std::nullopt;
  BehaviorLaneChangePreparePlan prepare;
  prepare.prepare_path.assign(
      reference.points.begin() + static_cast<std::ptrdiff_t>(start_index),
      reference.points.begin() + static_cast<std::ptrdiff_t>(end_index + 1));
  prepare_point_count = prepare.prepare_path.size();
  const double velocity = std::max(
      0.1, static_cast<double>(prepare.prepare_path.back()
                                   .longitudinal_velocity_mps));
  prepare.phase.length_m = prepare_length;
  prepare.phase.duration_s = prepare_length / velocity;
  prepare.phase.terminal_velocity_mps = velocity;
  const auto &start = prepare.prepare_path.back();
  const auto target_start = nearestIndex(
      request.target_reference, start.x, start.y);
  if (target_start + 2 >= request.target_reference.size()) return std::nullopt;
  std::size_t target_end = target_start;
  double shift_length = 0.0;
  while (target_end + 1 < request.target_reference.size() &&
         shift_length < request.shift_length_m) {
    shift_length += std::hypot(
        request.target_reference[target_end + 1].x -
            request.target_reference[target_end].x,
        request.target_reference[target_end + 1].y -
            request.target_reference[target_end].y);
    ++target_end;
  }
  if (target_end <= target_start + 1 ||
      shift_length + 0.5 < request.shift_length_m) return std::nullopt;
  const auto limit = regulatoryLimit(request);
  if (limit && prepare_length + shift_length > *limit + 1.0e-6)
    return std::nullopt;
  const auto &target_a = request.target_reference[target_start];
  const auto &target_b = request.target_reference[target_start + 1];
  const double dx = target_b.x - target_a.x;
  const double dy = target_b.y - target_a.y;
  const double norm = std::hypot(dx, dy);
  if (norm < 1.0e-6) return std::nullopt;
  BehaviorLaneChangeShiftPlan shift;
  shift.target_reference.assign(
      request.target_reference.begin() +
          static_cast<std::ptrdiff_t>(target_start),
      request.target_reference.end());
  shift.change_end = {request.target_reference[target_end].x,
                      request.target_reference[target_end].y};
  shift.lateral_shift_m =
      (dx * (start.y - target_a.y) -
       dy * (start.x - target_a.x)) / norm;
  shift.phase.length_m = shift_length;
  shift.phase.duration_s = shift_length / velocity;
  shift.phase.terminal_velocity_mps = velocity;
  shift.phase.lateral_acceleration_mps2 =
      request.lateral_acceleration_mps2;
  prepare.shifts.push_back(std::move(shift));
  BehaviorLaneChangeCandidateSearchInput search;
  search.prepare_plans.push_back(std::move(prepare));
  search.safety = request.safety;
  search.safety.prediction.initial_velocity_mps = std::max(
      0.0, static_cast<double>(ego.longitudinal_velocity_mps));
  search.safety.prediction.prepare_velocity_limit_mps = std::max(
      velocity, search.safety.prediction.initial_velocity_mps);
  search.safety.prediction.lane_change_velocity_limit_mps =
      search.safety.prediction.prepare_velocity_limit_mps;
  return searchBehaviorLaneChangeCandidates(search);
}

std::optional<BehaviorLaneChangeCandidateSearchResult>
searchSampledRouteLaneChange(
    const BehaviorGoalPath &reference, const TrajectoryPoint &ego,
    const BehaviorRouteLaneChangeRequest &request,
    std::size_t &prepare_point_count) {
  if (request.target_reference.size() < 3 ||
      request.shift_length_m <= 0.0) return std::nullopt;
  if (blockedBeforeRegulation(request)) return std::nullopt;
  const auto ego_index = nearestIndex(reference.points, ego.x, ego.y);
  if (ego_index + 2 >= reference.points.size()) return std::nullopt;
  const auto &settings = request.sampling;
  const double speed = std::max(
      0.0, static_cast<double>(ego.longitudinal_velocity_mps));
  const double path_velocity = std::max(
      0.0, static_cast<double>(reference.points[ego_index]
                                   .longitudinal_velocity_mps));
  double current_remaining = 0.0;
  for (std::size_t index = ego_index + 1;
       index < reference.points.size(); ++index) {
    current_remaining += std::hypot(
        reference.points[index].x - reference.points[index - 1].x,
        reference.points[index].y - reference.points[index - 1].y);
  }
  if (current_remaining < 2.0) return std::nullopt;
  const double nominal_duration = laneChangeActualPrepareDuration(
      speed, settings.minimum_change_velocity_mps,
      settings.maximum_longitudinal_acceleration_mps2,
      settings.minimum_prepare_duration_s,
      settings.maximum_prepare_duration_s,
      settings.active_signal_duration_s);
  const auto durations = laneChangePrepareDurations(
      current_remaining, settings.front_overhang_m,
      settings.signal_activation_length_m, nominal_duration,
      settings.maximum_prepare_duration_s);
  double target_remaining = 0.0;
  for (std::size_t index = 1;
       index < request.target_reference.size(); ++index) {
    target_remaining += std::hypot(
        request.target_reference[index].x -
            request.target_reference[index - 1].x,
        request.target_reference[index].y -
            request.target_reference[index - 1].y);
  }
  const bool sample_acceleration =
      settings.force_acceleration_sampling ||
      laneChangeNeedsAccelerationSampling(
          settings.maximum_longitudinal_acceleration_mps2, false,
          request.shift_length_m, current_remaining,
          target_remaining);
  BehaviorLaneChangeCandidateSearchInput search;
  search.prepare_length_threshold_m = 1.0;
  search.shift_length_threshold_m = 1.0;
  search.safety = request.safety;
  search.safety.prediction.initial_velocity_mps = speed;
  search.safety.prediction.prepare_velocity_limit_mps =
      std::max({path_velocity, settings.minimum_change_velocity_mps,
                speed});
  search.safety.prediction.lane_change_velocity_limit_mps =
      search.safety.prediction.prepare_velocity_limit_mps;
  for (const double duration : durations) {
    const auto phases = generateLaneChangePreparePhases(
        speed, path_velocity,
        settings.maximum_vehicle_velocity_mps,
        settings.minimum_change_velocity_mps, duration,
        settings.minimum_longitudinal_acceleration_mps2,
        settings.maximum_longitudinal_acceleration_mps2,
        settings.longitudinal_acceleration_samples,
        sample_acceleration, 1.0,
        current_remaining - 1.0);
    for (const auto &phase : phases) {
      std::size_t end_index = ego_index;
      double distance = 0.0;
      while (end_index + 1 < reference.points.size() &&
             distance < phase.length_m) {
        distance += std::hypot(
            reference.points[end_index + 1].x -
                reference.points[end_index].x,
            reference.points[end_index + 1].y -
                reference.points[end_index].y);
        ++end_index;
      }
      if (end_index <= ego_index ||
          distance + 0.5 < phase.length_m) continue;
      BehaviorLaneChangePreparePlan prepare;
      prepare.phase = phase;
      prepare.prepare_path.assign(
          reference.points.begin() +
              static_cast<std::ptrdiff_t>(ego_index),
          reference.points.begin() +
              static_cast<std::ptrdiff_t>(end_index + 1));
      const auto &start = prepare.prepare_path.back();
      const auto target_start = nearestIndex(
          request.target_reference, start.x, start.y);
      if (target_start + 2 >= request.target_reference.size())
        continue;
      const auto &a = request.target_reference[target_start];
      const auto &b = request.target_reference[target_start + 1];
      const double dx = b.x - a.x;
      const double dy = b.y - a.y;
      const double norm = std::hypot(dx, dy);
      if (norm < 1.0e-6) continue;
      const double lateral =
          (dx * (start.y - a.y) - dy * (start.x - a.x)) /
          norm;
      const bool near_terminal =
          current_remaining < phase.length_m +
              request.shift_length_m;
      const auto shift_phases = generateLaneChangeShiftPhases(
          lateral, phase.terminal_velocity_mps,
          path_velocity, settings.maximum_vehicle_velocity_mps,
          settings.minimum_change_velocity_mps,
          phase.actual_longitudinal_acceleration_mps2,
          settings.minimum_lateral_acceleration_mps2,
          settings.maximum_lateral_acceleration_mps2,
          settings.lateral_acceleration_samples,
          settings.lateral_jerk_mps3,
          request.shift_length_m, near_terminal,
          settings.deceleration_factor);
      for (const auto &shift_phase : shift_phases) {
        const auto limit = regulatoryLimit(request);
        if (limit && phase.length_m + shift_phase.length_m >
                         *limit + 1.0e-6)
          continue;
        std::size_t target_end = target_start;
        double shift_distance = 0.0;
        while (target_end + 1 < request.target_reference.size() &&
               shift_distance < shift_phase.length_m) {
          shift_distance += std::hypot(
              request.target_reference[target_end + 1].x -
                  request.target_reference[target_end].x,
              request.target_reference[target_end + 1].y -
                  request.target_reference[target_end].y);
          ++target_end;
        }
        if (target_end <= target_start + 1 ||
            shift_distance + 0.5 < shift_phase.length_m)
          continue;
        BehaviorLaneChangeShiftPlan shift;
        shift.phase = shift_phase;
        shift.target_reference.assign(
            request.target_reference.begin() +
                static_cast<std::ptrdiff_t>(target_start),
            request.target_reference.end());
        shift.change_end = {
            request.target_reference[target_end].x,
            request.target_reference[target_end].y};
        shift.lateral_shift_m = lateral;
        prepare.shifts.push_back(std::move(shift));
      }
      if (!prepare.shifts.empty())
        search.prepare_plans.push_back(std::move(prepare));
    }
  }
  if (search.prepare_plans.empty()) return std::nullopt;
  auto result = searchBehaviorLaneChangeCandidates(search);
  if (!result.candidates.empty()) {
    const double selected_length = result.candidates[
        result.first_safe_index.value_or(0)].prepare_phase.length_m;
    std::size_t end_index = ego_index;
    double distance = 0.0;
    while (end_index + 1 < reference.points.size() &&
           distance < selected_length) {
      distance += std::hypot(
          reference.points[end_index + 1].x -
              reference.points[end_index].x,
          reference.points[end_index + 1].y -
              reference.points[end_index].y);
      ++end_index;
    }
    prepare_point_count = end_index - ego_index + 1;
  }
  return result;
}

}  // 익명 네임스페이스

double behaviorRouteAvoidanceSpeedThreshold(
    const BehaviorRouteAvoidancePerceptionParameters &parameters,
    std::uint8_t classification) {
  const double source_limit = classification == 0 ||
      classification == 6 || classification == 7 ? 0.28 : 1.0;
  return std::min(parameters.stationary_speed_mps, source_limit);
}

double behaviorRouteAvoidanceStopThreshold(
    const BehaviorRouteAvoidancePerceptionParameters &parameters,
    std::uint8_t classification) {
  const double source_limit = classification == 0 ? 3.0 :
      classification >= 5 ? 1.0 : 2.0;
  return std::max(parameters.stationary_time_s, source_limit);
}

bool behaviorAvoidanceByLaneChangeRequested(
    const BehaviorRouteLaneChangeRequest::AvoidanceActivation &input) {
  if (input.objects.empty() ||
      !std::isfinite(input.minimum_lane_change_buffer_m) ||
      input.minimum_lane_change_buffer_m < 0.0)
    return false;
  bool has_target = false;
  const BehaviorRouteLaneChangeRequest::AvoidanceObject *nearest = nullptr;
  for (const auto &object : input.objects) {
    if (!std::isfinite(object.longitudinal_distance_m) ||
        !std::isfinite(object.minimum_avoidance_distance_m) ||
        object.minimum_avoidance_distance_m < 0.0 ||
        object.classification >= input.target_types.size())
      return false;
    if (!nearest || object.longitudinal_distance_m <
                        nearest->longitudinal_distance_m)
      nearest = &object;
    if (object.avoid_required &&
        input.target_types[object.classification])
      has_target = true;
  }
  return has_target && nearest &&
      nearest->longitudinal_distance_m > std::max(
          nearest->minimum_avoidance_distance_m,
          input.minimum_lane_change_buffer_m);
}

double behaviorMinimumLaneChangeDistanceBuffer(
    double lateral_shift_m, double minimum_velocity_mps,
    double lateral_jerk_mps3,
    double maximum_lateral_acceleration_mps2,
    double finish_judge_buffer_m) {
  if (!std::isfinite(lateral_shift_m) ||
      !std::isfinite(minimum_velocity_mps) ||
      !std::isfinite(lateral_jerk_mps3) ||
      !std::isfinite(maximum_lateral_acceleration_mps2) ||
      !std::isfinite(finish_judge_buffer_m) ||
      minimum_velocity_mps <= 0.0 ||
      lateral_jerk_mps3 <= 0.0 ||
      maximum_lateral_acceleration_mps2 <= 0.0 ||
      finish_judge_buffer_m < 0.0)
    return std::numeric_limits<double>::infinity();
  const double jerk = lateral_jerk_mps3;
  const double acceleration = maximum_lateral_acceleration_mps2;
  const double shift = std::abs(lateral_shift_m);
  double jerk_time = acceleration / jerk;
  double acceleration_time = (
      std::sqrt(acceleration * acceleration +
                4.0 * jerk * jerk * shift / acceleration) -
      3.0 * acceleration) / (2.0 * jerk);
  if (acceleration_time < 0.0) {
    jerk_time = std::cbrt(shift / (2.0 * jerk));
    acceleration_time = 0.0;
  }
  return minimum_velocity_mps *
      (4.0 * jerk_time + 2.0 * acceleration_time) +
      finish_judge_buffer_m;
}

std::vector<StaticAvoidanceSingleObjectInput>
makeBehaviorRouteAvoidanceObjects(
    const Trajectory &reference, const TrajectoryPoint &ego,
    const std::vector<std::pair<double, double>> &left_boundary,
    const std::vector<std::pair<double, double>> &right_boundary,
    const std::vector<BehaviorRouteObservedObject> &objects,
    const BehaviorRouteAvoidancePerceptionParameters &parameters) {
  std::vector<StaticAvoidanceSingleObjectInput> selected;
  if (reference.size() < 3 || left_boundary.size() < 2 ||
      right_boundary.size() < 2 ||
      parameters.nominal_lateral_jerk_mps3 <= 0.0 ||
      parameters.minimum_distance_jerk_mps3 <= 0.0 ||
      parameters.maximum_lateral_jerk_mps3 <= 0.0 ||
      !std::isfinite(parameters.ambiguous_vehicle_wait_s) ||
      parameters.ambiguous_vehicle_wait_s < 0.0 ||
      !std::isfinite(parameters.ambiguous_vehicle_maximum_movement_m) ||
      parameters.ambiguous_vehicle_maximum_movement_m < 0.0 ||
      !std::isfinite(parameters.intersection_yaw_deviation_rad) ||
      parameters.intersection_yaw_deviation_rad < 0.0 ||
      parameters.intersection_yaw_deviation_rad >=
          std::acos(-1.0) * 0.5 ||
      !std::isfinite(parameters.parked_center_offset_m) ||
      parameters.parked_center_offset_m < 0.0 ||
      !std::isfinite(parameters.parked_shiftable_ratio) ||
      parameters.parked_shiftable_ratio < 0.0 ||
      !std::isfinite(parameters.minimum_road_shoulder_width_m) ||
      parameters.minimum_road_shoulder_width_m < 0.0 ||
      !std::isfinite(parameters.parked_lateral_hard_margin_m) ||
      parameters.parked_lateral_hard_margin_m < 0.0) return selected;
  const auto ego_index = nearestIndex(reference, ego.x, ego.y);
  std::vector<double> arcs(reference.size(), 0.0);
  for (std::size_t index = 1; index < reference.size(); ++index) {
    arcs[index] = arcs[index - 1] + std::hypot(
        reference[index].x - reference[index - 1].x,
        reference[index].y - reference[index - 1].y);
  }
  for (const auto &object : objects) {
    if (!std::isfinite(object.x) || !std::isfinite(object.y) ||
        !std::isfinite(object.yaw_rad) ||
        !std::isfinite(object.length_m) ||
        !std::isfinite(object.width_m) ||
        !std::isfinite(object.speed_mps) ||
        !std::isfinite(object.stationary_duration_s) ||
        !std::isfinite(object.initial_displacement_m) ||
        object.length_m <= 0.0 || object.width_m <= 0.0 ||
        object.classification > 7 ||
        std::abs(object.speed_mps) > behaviorRouteAvoidanceSpeedThreshold(
            parameters, object.classification) ||
        object.stationary_duration_s < behaviorRouteAvoidanceStopThreshold(
            parameters, object.classification))
      continue;
    if ((object.classification == 6 || object.classification == 7) &&
        (object.within_crosswalk || object.within_intersection))
      continue;
    const auto object_index = nearestIndex(
        reference, object.x, object.y);
    const double lateral = signedLateral(
        reference[object_index], object.x, object.y);
    const auto footprint = object.footprint.empty() ?
        std::vector<std::pair<double, double>>{
            {-object.length_m / 2.0, -object.width_m / 2.0},
            {object.length_m / 2.0, -object.width_m / 2.0},
            {object.length_m / 2.0, object.width_m / 2.0},
            {-object.length_m / 2.0, object.width_m / 2.0}} :
        object.footprint;
    const double cosine = std::cos(object.yaw_rad);
    const double sine = std::sin(object.yaw_rad);
    BehaviorSafetyPolygon world_polygon;
    double nearest_lateral = std::abs(lateral);
    for (const auto &[x, y] : footprint) {
      const double world_x = object.x + x * cosine - y * sine;
      const double world_y = object.y + x * sine + y * cosine;
      world_polygon.emplace_back(world_x, world_y);
      nearest_lateral = std::min(nearest_lateral,
          std::abs(signedLateral(
              reference[object_index], world_x, world_y)));
    }
    if ((object.classification != 0 &&
         std::abs(lateral) < 0.05) ||
        nearest_lateral > parameters.maximum_lateral_distance_m)
      continue;
    if (object.classification != 0 &&
        object.on_current_road_lane && object.has_side_road_lane)
      continue;
    const bool vehicle_type = object.classification >= 1 &&
        object.classification <= 5;
    const bool object_on_right = lateral < 0.0;
    const auto &nearest_pose = reference[object_index];
    const double path_yaw = std::atan2(
        2.0 * (nearest_pose.orientation_w * nearest_pose.orientation_z +
               nearest_pose.orientation_x * nearest_pose.orientation_y),
        1.0 - 2.0 *
            (nearest_pose.orientation_y * nearest_pose.orientation_y +
             nearest_pose.orientation_z * nearest_pose.orientation_z));
    const double yaw_deviation = std::remainder(
        object.yaw_rad - path_yaw, 2.0 * std::acos(-1.0));
    const bool parallel =
        std::abs(yaw_deviation) <
            parameters.intersection_yaw_deviation_rad ||
        std::abs(yaw_deviation) > std::acos(-1.0) -
            parameters.intersection_yaw_deviation_rad;
    const bool merging = object_on_right ?
        !((yaw_deviation < 0.0 &&
           yaw_deviation > -std::acos(-1.0) * 0.5) ||
          yaw_deviation > std::acos(-1.0) * 0.5) :
        !((yaw_deviation > 0.0 &&
           yaw_deviation < std::acos(-1.0) * 0.5) ||
          yaw_deviation < -std::acos(-1.0) * 0.5);
    const bool deviating = !parallel && !merging;
    bool parked = false;
    if (vehicle_type && !object.within_intersection &&
        std::abs(lateral) >= parameters.parked_center_offset_m) {
      const auto &near_boundary = object_on_right ?
          right_boundary : left_boundary;
      const double center_to_boundary = boundaryDistance(
          near_boundary, reference[object_index].x,
          reference[object_index].y);
      const double shiftable = center_to_boundary -
          object.width_m * 0.5 +
          parameters.minimum_road_shoulder_width_m;
      parked = std::isfinite(shiftable) && shiftable > 0.0 &&
          std::abs(lateral) / shiftable >
              parameters.parked_shiftable_ratio;
    }
    if (vehicle_type && object.on_current_road_lane && !parked &&
        ((object.within_intersection && !deviating) ||
         object.initial_displacement_m >
             parameters.ambiguous_vehicle_maximum_movement_m ||
         object.stationary_duration_s <=
             parameters.ambiguous_vehicle_wait_s))
      continue;
    StaticAvoidanceSingleObjectInput input;
    input.classification = object.classification;
    input.ego_x = ego.x;
    input.ego_y = ego.y;
    input.closest_reference_pose = reference[object_index];
    input.object_on_right = object_on_right;
    const bool vulnerable = object.classification >= 6;
    const double hard_margin = object.classification == 0 ? -0.2 :
        vulnerable ? 0.5 : object.classification == 5 ? 0.3 :
        parameters.lateral_hard_margin_m;
    const double soft_margin = vulnerable || object.classification == 0 ?
        std::max(0.7, parameters.lateral_soft_margin_m) :
        parameters.lateral_soft_margin_m;
    input.envelope_buffer_m = object.classification >= 5 ? 0.5 : 0.1;
    input.object_polygon = std::move(world_polygon);
    const auto span = staticAvoidanceEnvelopeLongitudinalSpan(
        reference, ego.x, ego.y, input.object_polygon);
    if (!span) continue;
    const double forward = span->start_from_ego_m;
    if (forward < -parameters.maximum_backward_distance_m ||
        forward > parameters.maximum_forward_distance_m)
      continue;
    if (parameters.goal_distance_from_ego_m) {
      const double goal_distance = *parameters.goal_distance_from_ego_m;
      if (forward > goal_distance ||
          (!parameters.allow_goal_modification &&
           forward + span->length_m / 2.0 +
               parameters.goal_clearance_m > goal_distance))
        continue;
    }
    input.boundary = StaticAvoidanceBoundaryInput{
        input.object_on_right ? left_boundary : right_boundary,
        path_yaw};
    input.margin.vehicle_width_m = parameters.vehicle_width_m;
    input.margin.distance_factor = 1.0;
    input.margin.lateral_hard_margin_m = hard_margin;
    input.margin.parked_lateral_hard_margin_m =
        object.classification <= 4 && vehicle_type ?
            parameters.parked_lateral_hard_margin_m : hard_margin;
    input.margin.parked = parked;
    input.margin.lateral_soft_margin_m = soft_margin;
    input.margin.soft_drivable_bound_margin_m =
        parameters.drivable_bound_margin_m;
    input.margin.hard_drivable_bound_margin_m =
        parameters.drivable_bound_margin_m;
    const double ego_speed = std::max(
        0.0, static_cast<double>(ego.longitudinal_velocity_mps));
    const double avoid_speed = std::max(
        ego_speed, parameters.nominal_avoidance_speed_mps);
    const double estimated_shift =
        parameters.vehicle_width_m / 2.0 +
        soft_margin + object.width_m / 2.0;
    const double nominal_distance = distanceForShift(
        estimated_shift, parameters.nominal_lateral_jerk_mps3,
        avoid_speed);
    input.feasibility.front_constant_distance_m =
        parameters.front_overhang_m;
    input.feasibility.prepare_distance_m = std::max(
        parameters.minimum_prepare_distance_m,
        avoid_speed * parameters.maximum_prepare_time_s);
    input.feasibility.nominal_avoidance_distance_m = nominal_distance;
    input.feasibility.avoidance_speed_mps = avoid_speed;
    input.feasibility.maximum_lateral_jerk_mps3 =
        parameters.maximum_lateral_jerk_mps3;
    input.feasibility.minimum_distance_jerk_mps3 =
        parameters.minimum_distance_jerk_mps3;
    input.feasibility.minimum_distance_speed_mps = avoid_speed;
    input.feasibility.lateral_execution_threshold_m = 0.09;
    input.feasibility.best_effort_lateral_margin = true;
    input.feasibility.shorten_margin_immediately = true;
    input.outline.front_constant_distance_m =
        parameters.front_overhang_m;
    input.outline.rear_constant_distance_m =
        parameters.rear_overhang_m + object.length_m;
    input.outline.maximum_return_distance_m = nominal_distance;
    input.outline.to_start_point_m =
        input.feasibility.prepare_distance_m;
    input.outline.to_return_point_m = 0.0;
    input.outline.skip_return =
        forward + input.outline.rear_constant_distance_m +
            nominal_distance > arcs.back() - arcs[ego_index];
    selected.push_back(std::move(input));
  }
  std::sort(selected.begin(), selected.end(),
            [&reference, &ego](const auto &first, const auto &second) {
    const auto first_span = staticAvoidanceEnvelopeLongitudinalSpan(
        reference, ego.x, ego.y, first.object_polygon);
    const auto second_span = staticAvoidanceEnvelopeLongitudinalSpan(
        reference, ego.x, ego.y, second.object_polygon);
    const double first_arc = first_span ?
        first_span->start_from_ego_m :
        std::numeric_limits<double>::infinity();
    const double second_arc = second_span ?
        second_span->start_from_ego_m :
        std::numeric_limits<double>::infinity();
    return first_arc < second_arc;
  });
  return selected;
}

bool behaviorPullOverPathCollides(
    const Trajectory &path, const TrajectoryPoint &ego,
    const BehaviorPathVehicleFootprint &vehicle,
    const std::vector<BehaviorSafetyPolygon> &objects,
    double margin_m, double maximum_deceleration_mps2,
    double maximum_extra_stopping_margin_m) {
  return pullOverPathCollides(
      path, ego, vehicle, objects, margin_m,
      maximum_deceleration_mps2,
      maximum_extra_stopping_margin_m);
}

BehaviorGoalPath selectBehaviorPullOverPhase(
    const BehaviorGoalPath &full_path,
    const std::vector<std::size_t> &phase_end_indices,
    const TrajectoryPoint &ego, double ego_speed_mps,
    double timestamp_s, BehaviorGoalPose goal,
    BehaviorPullOverPhaseState &state,
    double arrival_distance_m,
    double minimum_phase_duration_s) {
  if (full_path.points.empty() ||
      full_path.lane_ids.size() != full_path.points.size() ||
      phase_end_indices.empty() ||
      phase_end_indices.front() == 0 ||
      !std::is_sorted(phase_end_indices.begin(),
                      phase_end_indices.end()) ||
      std::adjacent_find(phase_end_indices.begin(),
                         phase_end_indices.end()) !=
          phase_end_indices.end() ||
      phase_end_indices.back() != full_path.points.size() - 1)
    return full_path;
  if (!state.goal ||
      std::hypot(state.goal->x - goal.x, state.goal->y - goal.y) > 0.01 ||
      std::abs(state.goal->yaw_rad - goal.yaw_rad) > 0.01 ||
      (state.phase_start_s && timestamp_s < *state.phase_start_s) ||
      state.index >= phase_end_indices.size()) {
    state = {};
    state.goal = goal;
  }
  if (!state.phase_start_s) state.phase_start_s = timestamp_s;
  const auto end_index = phase_end_indices[state.index];
  const auto &end = full_path.points[end_index];
  if (state.index + 1 < phase_end_indices.size() &&
      std::isfinite(timestamp_s) && std::isfinite(ego_speed_mps) &&
      timestamp_s - *state.phase_start_s >=
          std::max(0.0, minimum_phase_duration_s) &&
      std::abs(ego_speed_mps) <= 0.1 &&
      std::hypot(ego.x - end.x, ego.y - end.y) <=
          std::max(0.0, arrival_distance_m)) {
    ++state.index;
    state.phase_start_s = timestamp_s;
  }
  const auto start_index = state.index == 0 ? 0 :
      phase_end_indices[state.index - 1];
  const auto last_index = phase_end_indices[state.index];
  BehaviorGoalPath phase = full_path;
  phase.points.assign(full_path.points.begin() + start_index,
                      full_path.points.begin() + last_index + 1);
  phase.lane_ids.assign(full_path.lane_ids.begin() + start_index,
                        full_path.lane_ids.begin() + last_index + 1);
  if (!phase.points.empty()) phase.points.back().longitudinal_velocity_mps = 0.0;
  return phase;
}

BehaviorGoalPath selectBehaviorStartPhase(
    const BehaviorGoalPath &full_path,
    const std::vector<std::size_t> &phase_end_indices,
    const TrajectoryPoint &ego, double ego_speed_mps,
    double timestamp_s, BehaviorGoalPose goal,
    BehaviorRoutePhaseState &state,
    bool terminal_is_goal,
    double arrival_distance_m,
    double minimum_phase_duration_s) {
  auto selected = selectBehaviorPullOverPhase(
      full_path, phase_end_indices, ego, ego_speed_mps,
      timestamp_s, goal, state,
      arrival_distance_m, minimum_phase_duration_s);
  if (!terminal_is_goal &&
      state.index + 1 == phase_end_indices.size() &&
      !selected.points.empty() && !full_path.points.empty())
    selected.points.back().longitudinal_velocity_mps =
        full_path.points.back().longitudinal_velocity_mps;
  return selected;
}

BehaviorRouteActionResult composeBehaviorRouteAction(
    const BehaviorGoalPath &reference, const TrajectoryPoint &ego,
    const std::vector<std::vector<BehaviorPathPoint2d>> &lane_polygons,
    const BehaviorRouteActionInput &input,
    const std::vector<StaticAvoidanceAnchoredShiftArc> &registered) {
  BehaviorRouteActionResult result;
  result.path = reference;
  result.registered_shift_arcs = registered;
  if (reference.points.size() < 3 ||
      reference.lane_ids.size() != reference.points.size() ||
      lane_polygons.empty()) return result;

  const int requested = static_cast<int>(input.start.has_value()) +
      static_cast<int>(input.side_shift.has_value()) +
      static_cast<int>(input.avoidance.has_value() || !registered.empty()) +
      static_cast<int>(input.lane_change.has_value()) +
      static_cast<int>(input.pull_over.has_value());
  if (requested > 1) {
    // 원본 관리자 슬롯의 출발→정적 회피→차로 변경→목표 순서로 후보를 평가합니다.
    const auto evaluate = [&](BehaviorRouteActionInput one,
                              const std::vector<StaticAvoidanceAnchoredShiftArc> &arcs) {
      return composeBehaviorRouteAction(
          reference, ego, lane_polygons, one, arcs);
    };
    const auto merge = [&](const BehaviorRouteActionResult &candidate) {
      result.start_candidate_count += candidate.start_candidate_count;
      result.side_shift_candidate_count +=
          candidate.side_shift_candidate_count;
      if (candidate.side_shift_end_index)
        result.side_shift_end_index = candidate.side_shift_end_index;
      result.avoidance_candidate_count += candidate.avoidance_candidate_count;
      result.lane_change_candidate_count += candidate.lane_change_candidate_count;
      result.lane_change_collision_count += candidate.lane_change_collision_count;
      result.lane_change_other_unsafe_count +=
          candidate.lane_change_other_unsafe_count;
      result.pull_over_candidate_count += candidate.pull_over_candidate_count;
      if (candidate.lane_change_decision)
        result.lane_change_decision = candidate.lane_change_decision;
      if (candidate.lane_change_first_safe_index)
        result.lane_change_first_safe_index =
            candidate.lane_change_first_safe_index;
      if (candidate.lane_change_shift_end_index)
        result.lane_change_shift_end_index =
            candidate.lane_change_shift_end_index;
      result.lane_change_candidate_outside_lanes |=
          candidate.lane_change_candidate_outside_lanes;
    };
    const auto choose = [&](const BehaviorRouteActionResult &candidate)
        -> std::optional<BehaviorRouteActionResult> {
      merge(candidate);
      if (candidate.kind == BehaviorRouteActionKind::LaneFollow)
        return std::nullopt;
      auto selected = candidate;
      selected.start_candidate_count = result.start_candidate_count;
      selected.side_shift_candidate_count =
          result.side_shift_candidate_count;
      selected.side_shift_end_index = result.side_shift_end_index;
      selected.avoidance_candidate_count = result.avoidance_candidate_count;
      selected.lane_change_candidate_count =
          result.lane_change_candidate_count;
      selected.lane_change_collision_count =
          result.lane_change_collision_count;
      selected.lane_change_other_unsafe_count =
          result.lane_change_other_unsafe_count;
      selected.pull_over_candidate_count = result.pull_over_candidate_count;
      selected.lane_change_decision = result.lane_change_decision;
      selected.lane_change_first_safe_index =
          result.lane_change_first_safe_index;
      selected.lane_change_shift_end_index =
          result.lane_change_shift_end_index;
      selected.lane_change_candidate_outside_lanes =
          result.lane_change_candidate_outside_lanes;
      if (candidate.kind != BehaviorRouteActionKind::StaticAvoidance)
        selected.registered_shift_arcs = result.registered_shift_arcs;
      return selected;
    };
    if (input.start) {
      BehaviorRouteActionInput one;
      one.start = input.start;
      if (const auto chosen = choose(evaluate(one, {}))) return *chosen;
    }
    if (input.avoidance || !registered.empty()) {
      BehaviorRouteActionInput one;
      one.avoidance = input.avoidance;
      const auto candidate = evaluate(one, registered);
      result.registered_shift_arcs = candidate.registered_shift_arcs;
      if (const auto chosen = choose(candidate)) return *chosen;
    }
    if (input.lane_change) {
      BehaviorRouteActionInput one;
      one.lane_change = input.lane_change;
      if (const auto chosen = choose(evaluate(one, {}))) return *chosen;
    }
    if (input.side_shift) {
      BehaviorRouteActionInput one;
      one.side_shift = input.side_shift;
      if (const auto chosen = choose(evaluate(one, {}))) return *chosen;
    }
    if (input.pull_over) {
      BehaviorRouteActionInput one;
      one.pull_over = input.pull_over;
      if (const auto chosen = choose(evaluate(one, {}))) return *chosen;
    }
    return result;
  }

  const auto ego_index = nearestIndex(reference.points, ego.x, ego.y);
  if (input.side_shift) {
    const auto &request = *input.side_shift;
    if (!std::isfinite(request.forward_path_length_m) ||
        !std::isfinite(request.backward_path_length_m) ||
        !std::isfinite(request.input_path_interval_m) ||
        request.forward_path_length_m < 0.0 ||
        request.backward_path_length_m < 0.0 ||
        request.input_path_interval_m < 0.0)
      return result;
    if (std::abs(request.requested_lateral_offset_m -
                 request.current_lateral_offset_m) < 1.0e-4)
      return result;
    const auto candidate = generateBehaviorSideShiftPath(
        reference.points, ego_index, ego.longitudinal_velocity_mps,
        request.current_lateral_offset_m,
        request.requested_lateral_offset_m, request.parameters);
    if (candidate) {
      result.side_shift_candidate_count = 1;
      if (request.approved) {
        std::size_t begin = ego_index;
        std::size_t end = ego_index;
        const double backward_limit =
            request.backward_path_length_m +
            request.input_path_interval_m;
        double distance = 0.0;
        while (begin > 0 && distance < backward_limit) {
          distance += std::hypot(
              candidate->path[begin].x -
                  candidate->path[begin - 1].x,
              candidate->path[begin].y -
                  candidate->path[begin - 1].y);
          --begin;
        }
        distance = 0.0;
        while (end + 1 < candidate->path.size() &&
               distance < request.forward_path_length_m) {
          distance += std::hypot(
              candidate->path[end + 1].x -
                  candidate->path[end].x,
              candidate->path[end + 1].y -
                  candidate->path[end].y);
          ++end;
        }
        Trajectory cropped(
            candidate->path.begin() +
                static_cast<std::ptrdiff_t>(begin),
            candidate->path.begin() +
                static_cast<std::ptrdiff_t>(end + 1));
        if (cropped.size() < 2) return result;
        result.path = withGeometry(reference, std::move(cropped));
        result.kind = BehaviorRouteActionKind::SideShift;
        if (candidate->line.end_index >= begin &&
            candidate->line.end_index <= end)
          result.side_shift_end_index =
              candidate->line.end_index - begin;
      }
    }
    return result;
  }
  if (input.lane_change) {
    const auto &request = *input.lane_change;
    if (request.avoidance_activation &&
        !behaviorAvoidanceByLaneChangeRequested(
            *request.avoidance_activation))
      return result;
    std::size_t prepare_count = 0;
    const auto search = request.sampling.enabled ?
        searchSampledRouteLaneChange(
            reference, ego, request, prepare_count) :
        searchRouteLaneChange(
            reference, ego, request, prepare_count);
    auto execution = request.execution;
    if (search) execution.search = *search;
    execution.fallback_path = reference.points;
    const auto selected = executeBehaviorLaneChange(execution);
    result.lane_change_candidate_count = execution.search.candidates.size();
    for (const auto &candidate : execution.search.candidates) {
      if (candidate.safety.safe) continue;
      if (candidate.safety.collision) ++result.lane_change_collision_count;
      else ++result.lane_change_other_unsafe_count;
    }
    result.lane_change_first_safe_index =
        execution.search.first_safe_index;
    result.lane_change_decision = selected.decision;
    // NormalLaneChange::isValidPath checks each candidate pose against the
    // expanded current/target lanelets. The full-vehicle departure check is
    // used by other behavior modules, but rejects valid lane-change paths at
    // lanelet ends before the shift reaches the next lanelet.
    const bool lane_safe = !selected.selected_candidate_index ||
        behaviorPathPointsInsideLanes(selected.path, lane_polygons);
    result.lane_change_candidate_outside_lanes = !lane_safe;
    if (selected.path.size() >= 2 && lane_safe) {
      result.path = withGeometry(reference, selected.path);
      if (selected.selected_candidate_index) {
        result.kind = BehaviorRouteActionKind::LaneChange;
        const auto selected_index = *selected.selected_candidate_index;
        const auto shift_end = prepare_count - 1 +
            execution.search.candidates[selected_index]
                .path.shift_end_index;
        result.lane_change_shift_end_index = shift_end;
        for (std::size_t index = prepare_count;
             index < result.path.lane_ids.size(); ++index) {
          std::vector<std::int64_t> target_ids;
          if (request.target_reference_lane_ids.size() ==
              request.target_reference.size()) {
            const auto target_index = nearestIndex(
                request.target_reference, result.path.points[index].x,
                result.path.points[index].y);
            target_ids = request.target_reference_lane_ids[target_index];
          } else if (!request.target_lanelet_ids.empty()) {
            target_ids = {request.target_lanelet_ids.front()};
          }
          if (!target_ids.empty()) {
            if (index <= shift_end) {
              for (const auto lane_id : target_ids) {
                auto &ids = result.path.lane_ids[index];
                if (std::find(ids.begin(), ids.end(), lane_id) == ids.end())
                  ids.push_back(lane_id);
              }
            } else {
              result.path.lane_ids[index] = std::move(target_ids);
            }
          }
        }
      } else if (selected.decision.module_state ==
                 BehaviorLaneChangeState::Abort) {
        result.kind = BehaviorRouteActionKind::LaneChangeAbort;
      } else if (selected.decision.module_state ==
                 BehaviorLaneChangeState::Stop) {
        result.kind = BehaviorRouteActionKind::LaneChangeStop;
      }
    }
    return result;
  }
  const auto &avoidance = input.avoidance;
  if (avoidance || !registered.empty()) {
    BehaviorRouteAvoidanceRequest request;
    if (avoidance) request = *avoidance;
    std::vector<StaticAvoidanceShiftArc> proposals;
    for (const auto &object : request.objects) {
      const auto plan = planStaticAvoidanceSingleObject(
          reference.points, object);
      if (!plan) continue;
      ++result.avoidance_candidate_count;
      proposals.push_back(plan->outline.avoid);
      if (plan->outline.return_to_center)
        proposals.push_back(*plan->outline.return_to_center);
    }
    const auto plan = planStaticAvoidanceWithRegisteredShifts(
        reference.points, ego_index, request.ego_linear_shift_m,
        registered, proposals, request.approved, request.shift_parameters);
    if (plan && !plan->active_shift_arcs.empty() &&
        !firstBehaviorPathLaneDeparture(
            plan->path, lane_polygons, request.vehicle)) {
      result.path = withGeometry(reference, plan->path);
      result.kind = BehaviorRouteActionKind::StaticAvoidance;
      result.registered_shift_arcs = plan->registered_shift_arcs;
      return result;
    }
    if (plan && plan->active_shift_arcs.empty())
      result.registered_shift_arcs.clear();
  }

  if (input.pull_over) {
    const auto &request = *input.pull_over;
    const auto collides_with_stopped = [&](const Trajectory &path) {
      return pullOverPathCollides(
          path, ego, request.vehicle,
          request.path_stop_objects,
          request.object_hard_margin_m,
          request.maximum_deceleration_mps2,
          request.maximum_extra_stopping_margin_m);
    };
    std::optional<ShiftPullOverCandidate> candidate;
    const double jerk_step =
        (request.parameters.maximum_lateral_jerk_mps3 -
         request.parameters.minimum_lateral_jerk_mps3) /
        std::max(1, request.parameters.jerk_samples);
    for (int sample = 0;
         !request.use_bezier && !request.use_geometric &&
         !request.use_freespace &&
         sample <= request.parameters.jerk_samples; ++sample) {
      auto parameters = request.parameters;
      parameters.minimum_lateral_jerk_mps3 =
          request.parameters.minimum_lateral_jerk_mps3 +
          sample * jerk_step;
      parameters.maximum_lateral_jerk_mps3 =
          parameters.minimum_lateral_jerk_mps3;
      parameters.jerk_samples = 1;
      auto proposed = generateShiftPullOverCandidate(
          reference.points, ego, request.goal, parameters);
      if (!proposed) continue;
      ++result.pull_over_candidate_count;
      if (firstBehaviorPathLaneDeparture(
              proposed->path, lane_polygons, request.vehicle))
        continue;
      if (collides_with_stopped(proposed->path)) continue;
      candidate = std::move(proposed);
      break;
    }
    if (!candidate &&
        !request.use_geometric && !request.use_freespace &&
        (request.use_bezier || request.use_bezier_fallback)) {
      const auto bezier = generateBezierPullOverCandidates(
          reference.points, ego, request.goal, request.parameters);
      for (const auto &proposed : bezier) {
        ++result.pull_over_candidate_count;
        if (firstBehaviorPathLaneDeparture(
                proposed.path, lane_polygons, request.vehicle))
          continue;
        if (collides_with_stopped(proposed.path)) continue;
        candidate = proposed;
        break;
      }
    }
    if (!candidate &&
        !request.use_freespace &&
        (request.use_geometric || request.use_geometric_fallback)) {
      const bool curved_first = referenceCurvesNearGoal(
          reference.points, request.goal);
      for (int attempt = 0; attempt < 2; ++attempt) {
        auto parameters = request.geometric;
        if (attempt == 1) parameters.forward = !parameters.forward;
        for (int geometry = 0; geometry < 2; ++geometry) {
          const bool curved = (geometry == 0) == curved_first;
          auto proposed = !curved ?
              generateGeometricPullOverCandidate(
                  reference.points, ego, request.goal, parameters) :
              generateCurvedGeometricPullOverCandidate(
                  reference.points, ego, request.goal, parameters);
          if (!proposed) continue;
          ++result.pull_over_candidate_count;
          if (firstBehaviorPathLaneDeparture(
                  proposed->path, lane_polygons, request.vehicle))
            continue;
          if (collides_with_stopped(proposed->path)) continue;
          candidate = std::move(proposed);
          break;
        }
        if (candidate) break;
      }
    }
    if (!candidate && request.freespace_grid &&
        (request.use_freespace || request.use_freespace_fallback)) {
      auto proposed = generateFreespacePullOverCandidate(
          ego, request.goal, *request.freespace_grid,
          request.freespace);
      if (proposed) {
        ++result.pull_over_candidate_count;
        if (!collides_with_stopped(proposed->path))
          candidate = std::move(proposed);
      }
    }
    if (candidate) {
      if (request.approved && request.safe &&
          !pullOverPathCollides(
              candidate->path, ego, request.vehicle,
              request.path_moving_objects,
              request.object_hard_margin_m,
              request.maximum_deceleration_mps2,
              request.maximum_extra_stopping_margin_m)) {
        result.path = withGeometry(reference, candidate->path);
        result.pull_over_phase_end_indices =
            candidate->phase_end_indices;
        result.kind = BehaviorRouteActionKind::PullOver;
        if (!request.goal_lanelet_ids.empty()) {
          for (std::size_t index = candidate->shift_start_index;
               index < result.path.lane_ids.size(); ++index) {
            const auto &point = result.path.points[index];
            const auto nearest = nearestIndex(
                reference.points, point.x, point.y);
            const double lateral = signedLateral(
                reference.points[nearest], point.x, point.y);
            if (index >= candidate->shift_end_index ||
                std::abs(lateral) >=
                    0.5 * std::abs(candidate->lateral_shift_m))
              result.path.lane_ids[index] =
                  request.goal_lanelet_ids;
          }
        }
      }
    }
    return result;
  }

  if (input.sampling) {
    const auto &request = *input.sampling;
    if (request.require_unsafe_reference) {
      bool reference_unsafe = false;
      for (const auto &point : reference.points) {
        if (behaviorFootprintCollidesWithObjects(
                {0.0, point.x, point.y, poseYaw(point), 0.0},
                {request.vehicle.front_m, request.vehicle.back_m,
                 request.vehicle.half_width_m},
                request.obstacle_polygons,
                std::max(1.0e-9, request.object_hard_margin_m))) {
          reference_unsafe = true;
          break;
        }
      }
      if (!reference_unsafe) return result;
    }
    auto sampling = request.sampling;
    if (sampling.reference.empty()) sampling.reference = reference.points;
    const auto initial = projectBehaviorFrenetState(
        sampling.reference, ego);
    if (!initial) return result;
    sampling.initial = *initial;
    std::vector<BehaviorFrenetCandidate> candidates;
    const auto append_previous = [&](Trajectory path,
                                     double target_lateral) {
      if (path.size() < 2) return;
      BehaviorFrenetCandidate candidate;
      candidate.path = std::move(path);
      candidate.target_lateral_m = target_lateral;
      candidate.arc_lengths_m.push_back(0.0);
      for (std::size_t index = 1; index < candidate.path.size(); ++index) {
        const auto &previous = candidate.path[index - 1];
        const auto &point = candidate.path[index];
        const double segment = std::hypot(
            point.x - previous.x, point.y - previous.y);
        candidate.arc_lengths_m.push_back(
            candidate.arc_lengths_m.back() + segment);
        candidate.curvatures_per_m.push_back(segment > 1.0e-8 ?
            std::remainder(poseYaw(point) - poseYaw(previous),
                           2.0 * std::acos(-1.0)) / segment : 0.0);
      }
      candidate.curvatures_per_m.push_back(
          candidate.curvatures_per_m.back());
      candidates.push_back(std::move(candidate));
    };
    if (request.previous_path.size() >= 2 &&
        request.previous_path_reuse_points > 0) {
      const auto &previous = request.previous_path;
      const auto previous_end = projectBehaviorFrenetState(
          sampling.reference, previous.back());
      append_previous(previous, previous_end ?
          previous_end->lateral_m : 0.0);
      const std::size_t current = nearestIndex(
          previous, ego.x, ego.y);
      std::vector<double> arcs{0.0};
      for (std::size_t index = 1; index < previous.size(); ++index)
        arcs.push_back(arcs.back() + std::hypot(
            previous[index].x - previous[index - 1].x,
            previous[index].y - previous[index - 1].y));
      const double remaining = arcs.back() - arcs[current];
      if (remaining > 1.0e-6) {
        for (int division = 0;
             division <= request.previous_path_reuse_points;
             ++division) {
          const double target = arcs[current] + remaining *
              static_cast<double>(division) /
              request.previous_path_reuse_points;
          std::size_t end = current + 1;
          while (end + 2 < previous.size() &&
                 arcs[end] < target) ++end;
          Trajectory prefix(
              previous.begin(),
              previous.begin() + static_cast<std::ptrdiff_t>(end));
          if (prefix.empty()) continue;
          const auto future = projectBehaviorFrenetState(
              sampling.reference, prefix.back());
          if (!future) continue;
          auto extension_input = sampling;
          extension_input.initial = *future;
          auto extensions = generateBehaviorFrenetCandidates(
              extension_input);
          for (auto &extension : extensions) {
            if (extension.path.empty()) continue;
            auto fused = prefix;
            fused.insert(fused.end(), extension.path.begin(),
                         extension.path.end());
            append_previous(std::move(fused),
                            extension.target_lateral_m);
          }
        }
      }
    } else {
      candidates = generateBehaviorFrenetCandidates(sampling);
    }
    result.sampling_candidate_count = candidates.size();
    const double maximum_target_length = sampling.target_lengths_m.empty() ?
        0.0 : *std::max_element(
            sampling.target_lengths_m.begin(),
            sampling.target_lengths_m.end());
    double maximum_target_lateral = 0.0;
    for (const double lateral : sampling.target_lateral_positions_m)
      maximum_target_lateral = std::max(
          maximum_target_lateral, std::abs(lateral));
    double best_cost = std::numeric_limits<double>::infinity();
    for (const auto &candidate : candidates) {
      if (candidate.path.size() < 2 ||
          firstBehaviorPathLaneDeparture(
              candidate.path, lane_polygons, request.vehicle)) continue;
      if (!std::isfinite(request.maximum_curvature_per_m) ||
          request.maximum_curvature_per_m <= 0.0) continue;
      bool valid = true;
      double maximum_curvature = 0.0;
      for (const double curvature : candidate.curvatures_per_m) {
        maximum_curvature = std::max(
            maximum_curvature, std::abs(curvature));
        if (!std::isfinite(curvature) ||
            std::abs(curvature) >=
                request.maximum_curvature_per_m) valid = false;
      }
      if (!valid) continue;
      for (const auto &point : candidate.path) {
        if (behaviorFootprintCollidesWithObjects(
                {0.0, point.x, point.y, poseYaw(point), 0.0},
                {request.vehicle.front_m, request.vehicle.back_m,
                 request.vehicle.half_width_m},
                request.obstacle_polygons,
                std::max(1.0e-9, request.object_hard_margin_m))) {
          valid = false;
          break;
        }
      }
      if (!valid) continue;
      const double length = candidate.arc_lengths_m.back();
      if (length <= 1.0e-8) continue;
      const double length_cost = maximum_target_length / length;
      const double lateral_cost = maximum_target_lateral > 1.0e-8 ?
          std::abs(candidate.target_lateral_m) /
              maximum_target_lateral : 0.0;
      const double curvature_cost = maximum_curvature /
          request.maximum_curvature_per_m;
      const double cost = request.cost_weights[0] * length_cost +
          request.cost_weights[1] * lateral_cost +
          request.cost_weights[2] * curvature_cost;
      if (cost < best_cost) {
        best_cost = cost;
        result.path = withGeometry(reference, candidate.path);
        result.kind = BehaviorRouteActionKind::Sampling;
      }
    }
    return result;
  }

  if (!input.start) return result;
  const auto &request = *input.start;
  if (!request.start_pose_candidates.empty()) {
    for (const auto &pose : request.start_pose_candidates) {
      if (!std::isfinite(pose.x) || !std::isfinite(pose.y) ||
          !std::isfinite(pose.yaw_rad)) continue;
      const double distance = std::hypot(
          pose.x - ego.x, pose.y - ego.y);
      const double ego_yaw = poseYaw(ego);
      const double backward =
          (pose.x - ego.x) * std::cos(ego_yaw) +
          (pose.y - ego.y) * std::sin(ego_yaw);
      if (backward > 0.05 ||
          std::abs(std::remainder(
              pose.yaw_rad - ego_yaw,
              2.0 * std::acos(-1.0))) > 0.15)
        continue;
      auto one = input;
      one.start->start_pose_candidates.clear();
      const bool current_pose_candidate = distance <= 0.5;
      one.start->start = current_pose_candidate ?
          BehaviorPathShiftAnchor{ego.x, ego.y} :
          BehaviorPathShiftAnchor{pose.x, pose.y};
      TrajectoryPoint candidate_ego = ego;
      if (!current_pose_candidate) {
        candidate_ego.x = pose.x;
        candidate_ego.y = pose.y;
        candidate_ego.orientation_z =
            std::sin(pose.yaw_rad / 2.0);
        candidate_ego.orientation_w =
            std::cos(pose.yaw_rad / 2.0);
      }
      candidate_ego.longitudinal_velocity_mps = 0.0F;
      if (!current_pose_candidate)
        one.start->lateral_offset_m = signedLateral(
            reference.points[nearestIndex(
                reference.points, candidate_ego.x, candidate_ego.y)],
            candidate_ego.x, candidate_ego.y);
      auto candidate = composeBehaviorRouteAction(
          reference, candidate_ego, lane_polygons, one, registered);
      result.start_candidate_count += candidate.start_candidate_count;
      if (candidate.kind != BehaviorRouteActionKind::StartPullOut)
        continue;
      candidate.start_candidate_count = result.start_candidate_count;
      candidate.selected_start_pose = pose;
      if (current_pose_candidate) return candidate;
      Trajectory backward_path;
      const int samples = std::max(1, static_cast<int>(
          std::ceil(distance / 0.5)));
      for (int sample = 0; sample <= samples; ++sample) {
        const double ratio = static_cast<double>(sample) / samples;
        auto point = ego;
        point.x = ego.x + ratio * (pose.x - ego.x);
        point.y = ego.y + ratio * (pose.y - ego.y);
        point.longitudinal_velocity_mps = sample == samples ?
            0.0F : -1.0F;
        backward_path.push_back(point);
      }
      if (!request.use_geometric && !request.use_geometric_fallback &&
          !request.use_freespace && !request.use_freespace_fallback &&
          firstBehaviorPathLaneDeparture(
              backward_path, lane_polygons, request.vehicle))
        continue;
      if (pullOverPathCollides(
              backward_path, ego, request.vehicle,
              request.path_stop_objects,
              request.object_hard_margin_m)) continue;
      const auto join_index = nearestIndex(
          candidate.path.points, pose.x, pose.y);
      if (join_index + 1 >= candidate.path.points.size())
        continue;
      const auto backward_end = backward_path.size() - 1;
      backward_path.insert(backward_path.end(),
          candidate.path.points.begin() + join_index + 1,
          candidate.path.points.end());
      std::vector<std::size_t> phase_ends{backward_end};
      for (const auto end : candidate.start_phase_end_indices)
        if (end > join_index)
          phase_ends.push_back(backward_end + end - join_index);
      if (phase_ends.back() != backward_path.size() - 1)
        phase_ends.push_back(backward_path.size() - 1);
      candidate.path = withGeometry(reference, std::move(backward_path));
      candidate.start_phase_end_indices = std::move(phase_ends);
      return candidate;
    }
    return result;
  }
  if (!request.use_freespace && !request.use_geometric) {
    const auto candidates = generateStartPullOutCandidates(
        reference.points, request.start, request.lateral_offset_m,
        request.terminal_is_goal, request.parameters);
    result.start_candidate_count = candidates.size();
    const auto lane_safe = selectLaneSafeStartPullOutCandidates(
        candidates, lane_polygons, request.vehicle);
    for (const auto &candidate : lane_safe) {
      if (!candidate.shifted) continue;
      if (pullOverPathCollides(
              candidate.path, ego, request.vehicle,
              request.path_stop_objects,
              request.object_hard_margin_m)) continue;
      result.path = withGeometry(reference, candidate.path);
      result.kind = BehaviorRouteActionKind::StartPullOut;
      break;
    }
  }
  if (result.kind == BehaviorRouteActionKind::LaneFollow &&
      (request.use_geometric || request.use_geometric_fallback)) {
    auto parameters = request.geometric;
    parameters.forward = false;
    parameters.after_straight_distance_m = 0.0;
    const BehaviorGoalPose parked_pose{ego.x, ego.y, poseYaw(ego)};
    const bool curved_first = referenceCurvesNearGoal(
        reference.points, parked_pose);
    for (int geometry = 0; geometry < 2; ++geometry) {
      const bool curved = (geometry == 0) == curved_first;
      const auto reversed_parking = curved ?
          generateCurvedGeometricPullOverCandidate(
              reference.points, reference.points[ego_index],
              parked_pose, parameters) :
          generateGeometricPullOverCandidate(
              reference.points, reference.points[ego_index],
              parked_pose, parameters);
      if (!reversed_parking ||
          reversed_parking->phase_end_indices.size() < 2 ||
          reversed_parking->shift_start_index + 2 >=
              reversed_parking->path.size()) continue;
      ++result.start_candidate_count;
      const auto arc_start = reversed_parking->shift_start_index;
      const auto arc_end = reversed_parking->path.size() - 1;
      Trajectory combined(
          reversed_parking->path.begin() + arc_start,
          reversed_parking->path.end());
      std::reverse(combined.begin(), combined.end());
      for (auto &point : combined)
        point.longitudinal_velocity_mps =
            static_cast<float>(parameters.forward_velocity_mps);
      combined.front().longitudinal_velocity_mps = 0.0F;
      const auto seam_index = arc_end -
          reversed_parking->phase_end_indices[1];
      if (seam_index < combined.size())
        combined[seam_index].longitudinal_velocity_mps = 0.0F;
      const auto road_start = combined.back();
      const auto road_yaw = poseYaw(road_start);
      const auto nearest = nearestIndex(
          reference.points, road_start.x, road_start.y);
      for (std::size_t index = nearest;
           index < reference.points.size(); ++index) {
        const auto &point = reference.points[index];
        const double ahead =
            (point.x - road_start.x) * std::cos(road_yaw) +
            (point.y - road_start.y) * std::sin(road_yaw);
        if (ahead > 1.0e-6)
          combined.push_back(point);
      }
      if (combined.size() >= 3 &&
          !firstBehaviorPathLaneDeparture(
              combined, lane_polygons, request.vehicle) &&
          !pullOverPathCollides(
              combined, ego, request.vehicle,
              request.path_stop_objects,
              request.object_hard_margin_m)) {
        result.path = withGeometry(reference, std::move(combined));
        result.kind = BehaviorRouteActionKind::StartPullOut;
        if (seam_index > 0 && seam_index + 1 <
            result.path.points.size())
          result.start_phase_end_indices.push_back(seam_index);
        result.start_phase_end_indices.push_back(
            result.path.points.size() - 1);
        break;
      }
    }
  }
  if (result.kind == BehaviorRouteActionKind::LaneFollow &&
      request.freespace_grid &&
      (request.use_freespace || request.use_freespace_fallback) &&
      std::isfinite(request.freespace_join_distance_m) &&
      request.freespace_join_distance_m > 0.0) {
    std::size_t join_index = ego_index;
    double traveled = 0.0;
    while (join_index + 2 < reference.points.size() &&
           traveled < request.freespace_join_distance_m) {
      const auto &a = reference.points[join_index];
      const auto &b = reference.points[join_index + 1];
      traveled += std::hypot(b.x - a.x, b.y - a.y);
      ++join_index;
    }
    const auto &join = reference.points[join_index];
    const BehaviorGoalPose join_pose{
        join.x, join.y, poseYaw(join)};
    auto parameters = request.freespace;
    parameters.final_straight_distance_m = 0.0;
    parameters.use_backward_goal_approach = true;
    const auto free_path = generateFreespacePullOverCandidate(
        ego, join_pose, *request.freespace_grid, parameters);
    if (free_path && free_path->path.size() >= 2) {
      ++result.start_candidate_count;
      Trajectory combined = free_path->path;
      const auto free_end_index = combined.size() - 1;
      const bool arrived_backward = free_end_index > 0 &&
          combined[free_end_index - 1].longitudinal_velocity_mps < 0.0F;
      if (!arrived_backward)
        combined.back().longitudinal_velocity_mps =
            reference.points[join_index].longitudinal_velocity_mps;
      combined.insert(combined.end(),
                      reference.points.begin() + join_index + 1,
                      reference.points.end());
      if (!pullOverPathCollides(
              combined, ego, request.vehicle,
              request.path_stop_objects,
              request.object_hard_margin_m)) {
        result.path = withGeometry(reference, std::move(combined));
        result.kind = BehaviorRouteActionKind::StartPullOut;
        result.start_phase_end_indices = free_path->phase_end_indices;
        if (!result.start_phase_end_indices.empty()) {
          if (arrived_backward)
            result.start_phase_end_indices.push_back(
                result.path.points.size() - 1);
          else
            result.start_phase_end_indices.back() =
                result.path.points.size() - 1;
        }
      }
    }
  }
  return result;
}

}  // planning 네임스페이스
