// SPDX-License-Identifier: Apache-2.0
// 원본 순항 객체 생성기의 충돌점 전후 제외 규칙입니다.

#include "planning/obstacle_cruise_candidate.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <stdexcept>

namespace planning {

CruisePredictedPose currentCruiseObjectPose(
    const CruisePredictedPose &initial_pose,
    const std::vector<CruisePredictedPath> &predicted_paths,
    const double elapsed_s) {
  if (!std::isfinite(elapsed_s))
    throw std::invalid_argument("순항 객체 예측 시각이 유효하지 않습니다");
  if (predicted_paths.empty() || elapsed_s < 0.0) return initial_pose;
  const auto path = std::max_element(
      predicted_paths.begin(), predicted_paths.end(),
      [](const auto &a, const auto &b) {
        return a.confidence < b.confidence;
      });
  if (path->poses.size() < 2 || path->time_step_s <= 0.0 ||
      !std::isfinite(path->time_step_s))
    return initial_pose;
  constexpr double epsilon = 1e-6;
  for (std::size_t i = 1; i < path->poses.size(); ++i) {
    if (elapsed_s - epsilon >= path->time_step_s * i) continue;
    const double ratio = std::clamp(
        (elapsed_s - path->time_step_s * (i - 1)) / path->time_step_s,
        0.0, 1.0);
    const auto &a = path->poses[i - 1];
    const auto &b = path->poses[i];
    const double angle = std::remainder(
        b.yaw_rad - a.yaw_rad, 2.0 * std::acos(-1.0));
    return {a.x + (b.x - a.x) * ratio,
            a.y + (b.y - a.y) * ratio,
            std::remainder(a.yaw_rad + angle * ratio,
                           2.0 * std::acos(-1.0))};
  }
  return initial_pose;
}

std::optional<CruiseObjectProjection> projectCruiseObjectToTrajectory(
    const Trajectory &trajectory, const std::size_t ego_index,
    const CruiseObjectKinematics &object) {
  if (trajectory.size() < 2 || ego_index >= trajectory.size() ||
      !std::isfinite(object.x) || !std::isfinite(object.y) ||
      !std::isfinite(object.yaw_rad) ||
      !std::isfinite(object.local_velocity_x) ||
      !std::isfinite(object.local_velocity_y))
    throw std::invalid_argument("순항 객체 투영 입력이 유효하지 않습니다");
  std::size_t nearest = 0;
  double minimum = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < trajectory.size(); ++i) {
    const double dx = object.x - trajectory[i].x;
    const double dy = object.y - trajectory[i].y;
    const double squared = dx * dx + dy * dy;
    if (squared < minimum) {
      minimum = squared;
      nearest = i;
    }
  }
  if (nearest < ego_index) return std::nullopt;
  double front_distance = 0.0;
  for (std::size_t i = ego_index; i < nearest; ++i)
    front_distance += std::hypot(
        trajectory[i + 1].x - trajectory[i].x,
        trajectory[i + 1].y - trajectory[i].y);
  const auto &point = trajectory[nearest];
  const double trajectory_yaw = std::atan2(
      2.0 * (point.orientation_w * point.orientation_z +
             point.orientation_x * point.orientation_y),
      1.0 - 2.0 * (point.orientation_y * point.orientation_y +
                   point.orientation_z * point.orientation_z));
  const double angle = std::remainder(
      object.yaw_rad - trajectory_yaw, 2.0 * std::acos(-1.0));
  const double longitudinal = std::cos(angle) * object.local_velocity_x -
                              std::sin(angle) * object.local_velocity_y;
  const double lateral_velocity = std::sin(angle) * object.local_velocity_x +
                                  std::cos(angle) * object.local_velocity_y;
  const double cross = std::cos(trajectory_yaw) * (object.y - point.y) -
                       std::sin(trajectory_yaw) * (object.x - point.x);
  const double approach = cross > 0.0 ? -lateral_velocity : lateral_velocity;
  std::size_t segment = nearest + 1 == trajectory.size()
      ? nearest - 1 : nearest;
  if (nearest > 0 && nearest + 1 < trajectory.size()) {
    const double sx = trajectory[nearest + 1].x - point.x;
    const double sy = trajectory[nearest + 1].y - point.y;
    if ((object.x - point.x) * sx +
        (object.y - point.y) * sy <= 0.0)
      segment = nearest - 1;
  }
  const double dx = trajectory[segment + 1].x - trajectory[segment].x;
  const double dy = trajectory[segment + 1].y - trajectory[segment].y;
  const double length = std::hypot(dx, dy);
  const double lateral = length < 1e-9 ? 0.0 :
      (dx * (object.y - trajectory[segment].y) -
       dy * (object.x - trajectory[segment].x)) / length;
  return CruiseObjectProjection{
      front_distance, lateral, angle, longitudinal, approach};
}

bool passesObstacleCruiseRoughLateralFilter(
    const CruiseObjectProjection &projection,
    const double object_max_length_m, const double vehicle_width_m,
    const double maximum_lateral_margin_m,
    const double maximum_lateral_time_s) {
  if (!std::isfinite(object_max_length_m) || object_max_length_m < 0.0 ||
      !std::isfinite(vehicle_width_m) || vehicle_width_m < 0.0 ||
      !std::isfinite(maximum_lateral_margin_m) ||
      maximum_lateral_margin_m < 0.0 ||
      !std::isfinite(maximum_lateral_time_s) ||
      maximum_lateral_time_s < 0.0 ||
      !std::isfinite(projection.signed_lateral_offset_m) ||
      !std::isfinite(projection.approach_velocity_mps))
    throw std::invalid_argument("순항 객체 횡거리 필터 입력이 유효하지 않습니다");
  const double rough_lateral =
      std::abs(projection.signed_lateral_offset_m) -
      vehicle_width_m - object_max_length_m;
  if (rough_lateral <= maximum_lateral_margin_m) return true;
  return projection.approach_velocity_mps > 0.0 &&
      rough_lateral /
          std::max(1e-6, projection.approach_velocity_mps) <=
          maximum_lateral_time_s;
}

CruiseCandidateRejection classifyCruiseCandidate(
    const CruiseCandidateInput &input,
    const CruiseCandidateParameters &parameters) {
  const double values[] = {
      input.ego_speed_mps, input.longitudinal_speed_mps,
      input.approach_speed_mps, input.lateral_distance_m,
      input.heading_difference_rad, input.object_speed_mps,
      input.collision_overlap_s, parameters.maximum_lateral_margin_m,
      parameters.maximum_lateral_time_s,
      parameters.crossing_angle_threshold_rad,
      parameters.minimum_outside_speed_mps,
      parameters.minimum_outside_overlap_s,
      parameters.minimum_ego_speed_for_outside_mps};
  if (!std::all_of(std::begin(values), std::end(values),
                   [](double value) { return std::isfinite(value); }) ||
      input.lateral_distance_m < 0.0 ||
      parameters.maximum_lateral_margin_m < 0.0 ||
      parameters.maximum_lateral_time_s < 0.0 ||
      parameters.crossing_angle_threshold_rad < 0.0 ||
      parameters.minimum_outside_speed_mps < 0.0 ||
      parameters.minimum_outside_overlap_s < 0.0 ||
      parameters.minimum_ego_speed_for_outside_mps < 0.0)
    throw std::invalid_argument("순항 후보 입력이 유효하지 않습니다");
  if (!input.supported_class)
    return CruiseCandidateRejection::UnsupportedClass;
  if (!input.driving_forward)
    return CruiseCandidateRejection::ReverseMotion;
  if (input.longitudinal_speed_mps < 0.0)
    return CruiseCandidateRejection::OppositeDirection;
  if (parameters.maximum_lateral_margin_m < input.lateral_distance_m &&
      input.lateral_distance_m /
              std::max(1e-6, input.approach_speed_mps) >
          parameters.maximum_lateral_time_s)
    return CruiseCandidateRejection::TooFarLaterally;
  const double angle = std::abs(std::remainder(
      input.heading_difference_rad, 2.0 * std::acos(-1.0)));
  if (parameters.crossing_angle_threshold_rad < angle &&
      parameters.crossing_angle_threshold_rad < std::acos(-1.0) - angle)
    return CruiseCandidateRejection::CrossingTrajectory;
  if (input.lateral_distance_m >= 1e-6) {
    if (input.ego_speed_mps < parameters.minimum_ego_speed_for_outside_mps)
      return CruiseCandidateRejection::OutsideWhileEgoStopped;
    if (!input.outside_class)
      return CruiseCandidateRejection::OutsideUnsupportedClass;
    if (input.object_speed_mps < parameters.minimum_outside_speed_mps)
      return CruiseCandidateRejection::OutsideTooSlow;
  }
  if (!input.has_collision_points)
    return CruiseCandidateRejection::NoCollision;
  if (input.lateral_distance_m >= 1e-6) {
    if (input.collision_overlap_s < parameters.minimum_outside_overlap_s)
      return CruiseCandidateRejection::OutsideShortOverlap;
    if (!input.first_collision_in_front)
      return CruiseCandidateRejection::OutsideBehindEgo;
  }
  return CruiseCandidateRejection::None;
}

CruiseInsideHysteresis::CruiseInsideHysteresis(
    const double entry_speed_mps, const double keep_speed_mps)
    : entry_speed_mps_(entry_speed_mps),
      keep_speed_mps_(keep_speed_mps) {
  if (!std::isfinite(entry_speed_mps) || !std::isfinite(keep_speed_mps) ||
      entry_speed_mps < 0.0 || keep_speed_mps < 0.0 ||
      keep_speed_mps > entry_speed_mps)
    throw std::invalid_argument("순항 진입·유지 속도 임계값이 유효하지 않습니다");
}

bool CruiseInsideHysteresis::permits(
    const std::string &id, const double longitudinal_speed_mps) const {
  if (id.empty() || !std::isfinite(longitudinal_speed_mps))
    throw std::invalid_argument("순항 객체 ID 또는 속도가 유효하지 않습니다");
  return longitudinal_speed_mps >=
         (previous_ids_.count(id) ? keep_speed_mps_ : entry_speed_mps_);
}

void CruiseInsideHysteresis::replacePreviousCruiseIds(
    const std::vector<std::string> &ids) {
  previous_ids_ = std::unordered_set<std::string>(ids.begin(), ids.end());
}

void CruiseInsideHysteresis::reset() { previous_ids_.clear(); }

std::vector<CruiseYieldPair> findCruiseYieldPairs(
    const std::vector<CruiseYieldObject> &objects,
    const CruiseYieldParameters &p) {
  const double parameters[] = {
      p.stopped_velocity_threshold_mps, p.lateral_distance_threshold_m,
      p.maximum_lateral_gap_m, p.maximum_collision_time_s,
      p.outside_minimum_speed_mps, p.vehicle_length_m};
  if (!std::all_of(std::begin(parameters), std::end(parameters),
                   [](double value) { return std::isfinite(value) && value >= 0.0; }))
    throw std::invalid_argument("양보 순항 설정이 유효하지 않습니다");
  std::vector<std::size_t> moving, stopped;
  for (std::size_t i = 0; i < objects.size(); ++i) {
    const auto &object = objects[i];
    const double values[] = {
        object.x, object.y, object.local_velocity_x_mps,
        object.local_velocity_y_mps, object.longitudinal_velocity_mps,
        object.lateral_distance_m, object.distance_from_ego_m,
        object.heading_difference_rad};
    if (!std::all_of(std::begin(values), std::end(values),
                     [](double value) { return std::isfinite(value); }))
      throw std::invalid_argument("양보 순항 객체가 유효하지 않습니다");
    const double speed = std::hypot(
        object.local_velocity_x_mps, object.local_velocity_y_mps);
    if (speed > p.stopped_velocity_threshold_mps) {
      if (object.lateral_distance_m < p.lateral_distance_threshold_m)
        moving.push_back(i);
    } else if (object.lateral_distance_m <
               p.lateral_distance_threshold_m + p.maximum_lateral_gap_m) {
      stopped.push_back(i);
    }
  }
  const auto by_distance = [&](const std::size_t a, const std::size_t b) {
    return objects[a].distance_from_ego_m < objects[b].distance_from_ego_m;
  };
  std::sort(moving.begin(), moving.end(), by_distance);
  std::sort(stopped.begin(), stopped.end(), by_distance);
  std::vector<CruiseYieldPair> pairs;
  for (const auto moving_index : moving) {
    const auto &candidate = objects[moving_index];
    const double speed = std::hypot(
        candidate.local_velocity_x_mps, candidate.local_velocity_y_mps);
    if (!candidate.outside_cruise_class || candidate.crossing_trajectory ||
        speed < p.outside_minimum_speed_mps ||
        candidate.longitudinal_velocity_mps < 0.0)
      continue;
    for (const auto stopped_index : stopped) {
      const auto &blocker = objects[stopped_index];
      if (candidate.distance_from_ego_m <= p.vehicle_length_m ||
          candidate.distance_from_ego_m >= blocker.distance_from_ego_m)
        continue;
      const double lateral_gap = std::abs(
          candidate.lateral_distance_m - blocker.lateral_distance_m);
      const double longitudinal_gap =
          blocker.distance_from_ego_m - candidate.distance_from_ego_m;
      if (lateral_gap < p.maximum_lateral_gap_m &&
          longitudinal_gap / speed < p.maximum_collision_time_s)
        pairs.push_back({moving_index, stopped_index});
    }
  }
  return pairs;
}

}  // planning 네임스페이스
