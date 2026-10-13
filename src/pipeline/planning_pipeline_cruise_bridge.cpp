// SPDX-License-Identifier: Apache-2.0
// 기록된 객체의 현재 자세·외형 충돌을 종단 정지 후보로 전달합니다.

#include "planning/planning_pipeline_cruise_bridge.hpp"

#include "planning/obstacle_cruise_candidate.hpp"
#include "planning/obstacle_cruise_collision.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

namespace planning {
namespace {

double yaw(const TrajectoryPoint &point) {
  return std::atan2(
      2.0 * (point.orientation_w * point.orientation_z +
             point.orientation_x * point.orientation_y),
      1.0 - 2.0 * (point.orientation_y * point.orientation_y +
                   point.orientation_z * point.orientation_z));
}

std::optional<double> obstacleMaxLength(
    const PlanningPerceivedObject &object) {
  if (object.shape_type == 0 && object.length_m > 0.0 &&
      object.width_m > 0.0)
    return std::hypot(object.length_m * 0.5, object.width_m * 0.5);
  if (object.shape_type == 1 && object.length_m > 0.0)
    return object.length_m * 0.5;
  if (object.shape_type == 2 && object.footprint.size() >= 3) {
    double radius = 0.0;
    for (const auto &point : object.footprint)
      radius = std::max(radius, std::hypot(point.x, point.y));
    if (radius > 0.0 && std::isfinite(radius)) return radius;
  }
  return std::nullopt;
}

std::size_t nearestIndex(const Trajectory &path,
                         const TrajectoryPoint &ego) {
  std::size_t nearest = 0;
  double best = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0; index < path.size(); ++index) {
    const double distance = std::hypot(
        path[index].x - ego.x, path[index].y - ego.y);
    if (distance < best) {
      best = distance;
      nearest = index;
    }
  }
  return nearest;
}

double arcAtIndex(const Trajectory &path, const std::size_t index) {
  double arc = 0.0;
  for (std::size_t i = 0; i < index; ++i)
    arc += std::hypot(path[i + 1].x - path[i].x,
                      path[i + 1].y - path[i].y);
  return arc;
}

double arcAtPoint(const Trajectory &path, const double x, const double y) {
  double best = std::numeric_limits<double>::infinity();
  double result = 0.0;
  double accumulated = 0.0;
  for (std::size_t index = 0; index + 1 < path.size(); ++index) {
    const double dx = path[index + 1].x - path[index].x;
    const double dy = path[index + 1].y - path[index].y;
    const double length = std::hypot(dx, dy);
    if (length < 1.0e-9) continue;
    const double ratio = std::clamp(
        ((x - path[index].x) * dx + (y - path[index].y) * dy) /
            (length * length), 0.0, 1.0);
    const double distance = std::hypot(
        x - path[index].x - ratio * dx,
        y - path[index].y - ratio * dy);
    if (distance < best) {
      best = distance;
      result = accumulated + ratio * length;
    }
    accumulated += length;
  }
  return result;
}

std::optional<double> firstStopArc(
    const Trajectory &path, const std::size_t ego_index) {
  double arc = 0.0;
  for (std::size_t i = 1; i < path.size(); ++i) {
    arc += std::hypot(path[i].x - path[i - 1].x,
                      path[i].y - path[i - 1].y);
    if (i > ego_index && path[i].longitudinal_velocity_mps <= 0.0)
      return arc;
  }
  return std::nullopt;
}

CruisePredictedPose futurePose(
    const std::vector<CruiseTimedPose> &prediction,
    const double time_s) {
  if (prediction.empty()) return {};
  if (time_s <= prediction.front().time_from_object_stamp_s)
    return prediction.front().pose;
  for (std::size_t i = 1; i < prediction.size(); ++i) {
    if (time_s > prediction[i].time_from_object_stamp_s) continue;
    const double duration = prediction[i].time_from_object_stamp_s -
                            prediction[i - 1].time_from_object_stamp_s;
    const double ratio = duration < 1.0e-9 ? 0.0 :
        (time_s - prediction[i - 1].time_from_object_stamp_s) / duration;
    const auto &a = prediction[i - 1].pose;
    const auto &b = prediction[i].pose;
    return {a.x + ratio * (b.x - a.x),
            a.y + ratio * (b.y - a.y),
            a.yaw_rad + ratio * std::remainder(
                b.yaw_rad - a.yaw_rad, 2.0 * std::acos(-1.0))};
  }
  return prediction.back().pose;
}

bool isCruiseCandidate(
    const PlanningPerceivedObject &object,
    const CruisePredictedPose &pose,
    const CruiseObjectShape &shape,
    const CruiseObjectProjection &projection,
    const Trajectory &sampled,
    const CruisePredictedPose &ego_pose,
    const double precise_lateral_m,
    const double ego_speed_mps,
    const bool driving_forward,
    const double elapsed_s,
    const CruiseInsideHysteresis &history,
    const CruiseCandidateParameters &cruise_parameters,
    const CruiseVehicleGeometry &vehicle,
    CruiseCandidateDiagnostic *accepted_diagnostic) {
  if (!driving_forward || object.predicted_paths.empty() ||
      projection.longitudinal_velocity_mps < 0.0) return false;
  std::vector<const CruisePredictedPath *> ranked;
  ranked.reserve(object.predicted_paths.size());
  for (const auto &prediction : object.predicted_paths)
    ranked.push_back(&prediction);
  std::sort(ranked.begin(), ranked.end(),
            [](const auto *left, const auto *right) {
              return left->confidence > right->confidence;
            });
  const double radius = *obstacleMaxLength(object);
  std::vector<CruiseCollisionPoint> collision_points;
  std::size_t selected_rank = 0;
  const std::size_t prediction_limit = precise_lateral_m < 1.0e-6 ?
      1 : std::min(cruise_parameters.outside_predicted_path_count,
                   ranked.size());
  for (std::size_t index = 0; index < prediction_limit; ++index) {
    const auto samples = resampleCruisePredictedPath(
        *ranked[index], 0.1, 10.0);
    collision_points = findCruiseCollisionPoints(
        sampled, samples, shape, vehicle.front_m, vehicle.rear_m,
        vehicle.width_m,
        elapsed_s, 10.0, radius + 2.0 +
            std::hypot(vehicle.front_m + vehicle.rear_m,
                       vehicle.width_m * 0.5 + 1.0), true, ego_pose);
    if (!collision_points.empty()) {
      selected_rank = index;
      break;
    }
  }
  std::size_t object_index = 0;
  double closest = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0; index < sampled.size(); ++index) {
    const double distance = std::hypot(
        sampled[index].x - pose.x, sampled[index].y - pose.y);
    if (distance < closest) {
      closest = distance;
      object_index = index;
    }
  }
  double collision_arc = 0.0;
  if (!collision_points.empty()) {
    const auto first_index = collision_points.front().trajectory_index;
    const auto lo = std::min(object_index, first_index);
    const auto hi = std::max(object_index, first_index);
    for (std::size_t index = lo; index < hi; ++index)
      collision_arc += std::hypot(
          sampled[index + 1].x - sampled[index].x,
          sampled[index + 1].y - sampled[index].y);
    if (first_index < object_index) collision_arc = -collision_arc;
  }
  CruiseCandidateInput decision;
  decision.supported_class = object.classification <= 6;
  decision.driving_forward = driving_forward;
  decision.outside_class = object.classification >= 1 &&
                           object.classification <= 5;
  decision.ego_speed_mps = ego_speed_mps;
  decision.longitudinal_speed_mps = projection.longitudinal_velocity_mps;
  decision.approach_speed_mps = projection.approach_velocity_mps;
  decision.lateral_distance_m = precise_lateral_m;
  decision.heading_difference_rad = projection.heading_difference_rad;
  decision.object_speed_mps = std::hypot(
      object.velocity_x_mps, object.velocity_y_mps);
  decision.has_collision_points = !collision_points.empty();
  decision.collision_overlap_s = collision_points.empty() ? 0.0 :
      collision_points.back().time_from_object_stamp_s -
      collision_points.front().time_from_object_stamp_s;
  decision.first_collision_in_front = collision_arc > -radius;
  const bool accepted = classifyCruiseCandidate(decision, cruise_parameters) ==
             CruiseCandidateRejection::None &&
         (precise_lateral_m >= 1.0e-6 ||
          history.permits(object.id, projection.longitudinal_velocity_mps));
  if (accepted && accepted_diagnostic && !collision_points.empty()) {
    *accepted_diagnostic = {
        object.id, precise_lateral_m, decision.collision_overlap_s,
        arcAtPoint(sampled, collision_points.front().x,
                   collision_points.front().y),
        projection.longitudinal_velocity_mps,
        projection.approach_velocity_mps, selected_rank};
  }
  return accepted;
}

}  // 익명 네임스페이스

void CruiseSlowdownCandidateState::beginFrame() {
  current_active_.clear();
  current_seen_.clear();
}

bool CruiseSlowdownCandidateState::shouldSlowDown(
    const std::string &id, const double lateral) {
  current_seen_.insert(id);
  const bool previous = previous_active_.count(id) != 0;
  const bool low = previous ? lateral <= 1.2 : lateral < 1.0;
  if (previous) {
    if (!low) {
      auto [it, inserted] = counters_.emplace(id, -1);
      if (!inserted) it->second = std::min(-1, it->second - 1);
      if (it->second <= -5) return false;
    }
    return true;
  }
  if (!low) return false;
  auto [it, inserted] = counters_.emplace(id, 1);
  if (!inserted) it->second = std::max(1, it->second + 1);
  return it->second >= 5;
}

void CruiseSlowdownCandidateState::accepted(const std::string &id) {
  current_active_.insert(id);
}

void CruiseSlowdownCandidateState::endFrame() {
  for (auto it = counters_.begin(); it != counters_.end();) {
    if (current_seen_.count(it->first) == 0) it = counters_.erase(it);
    else ++it;
  }
  previous_active_ = current_active_;
}

void CruiseSlowdownCandidateState::reset() {
  counters_.clear();
  previous_active_.clear();
  current_active_.clear();
  current_seen_.clear();
}

PlanningCruiseStopCandidateCore::PlanningCruiseStopCandidateCore(
    CruiseCandidateParameters cruise_parameters,
    CruiseVehicleGeometry vehicle)
    : cruise_parameters_(cruise_parameters), vehicle_(vehicle),
      cruise_history_(cruise_parameters.inside_cruise_entry_speed_mps,
                      cruise_parameters.inside_cruise_keep_speed_mps) {
  if (cruise_parameters.outside_predicted_path_count == 0)
    throw std::invalid_argument("차로 밖 순항 예측 경로 수는 1 이상이어야 합니다");
  if (!std::isfinite(cruise_parameters.stop_safe_distance_margin_m) ||
      cruise_parameters.stop_safe_distance_margin_m < 0.0)
    throw std::invalid_argument("장애물 정지 안전거리 여유가 유효하지 않습니다");
  if (!std::isfinite(cruise_parameters.stop_collision_time_margin_s) ||
      cruise_parameters.stop_collision_time_margin_s < 0.0)
    throw std::invalid_argument("정지 충돌 시간 여유는 유효한 0 이상이어야 합니다");
  if (cruise_parameters.outside_stop_predicted_path_count == 0 ||
      !std::isfinite(cruise_parameters.outside_stop_lateral_time_s) ||
      cruise_parameters.outside_stop_lateral_time_s < 0.0 ||
      !std::isfinite(cruise_parameters.pedestrian_deceleration_mps2) ||
      cruise_parameters.pedestrian_deceleration_mps2 <= 0.0 ||
      !std::isfinite(cruise_parameters.bicycle_deceleration_mps2) ||
      cruise_parameters.bicycle_deceleration_mps2 <= 0.0)
    throw std::invalid_argument("차로 밖 정지 예측 설정이 유효하지 않습니다");
}

std::vector<ObstacleCruiseStopCandidate>
PlanningCruiseStopCandidateCore::plan(
    const Trajectory &trajectory, const TrajectoryPoint &ego,
    const double ego_speed_mps, const double now_s,
    const PlanningCruisePerceptionFrame &perception) {
  claimed_object_ids_.clear();
  pid_obstacles_.clear();
  cruise_diagnostics_.clear();
  if (trajectory.size() < 2) return {};
  // motion_utils::isDrivingForwardWithTwist uses the first two poses when
  // the input has at least two points, regardless of their velocity values.
  const double heading = yaw(trajectory.front());
  const double path_heading = std::atan2(
      trajectory[1].y - trajectory[0].y,
      trajectory[1].x - trajectory[0].x);
  const bool driving_forward =
      std::abs(std::remainder(heading - path_heading,
                              2.0 * std::acos(-1.0))) <
      std::acos(-1.0) * 0.5;
  const auto ego_index = nearestIndex(trajectory, ego);
  const auto segment = std::min(ego_index, trajectory.size() - 2);
  const auto sampled = resampleCruiseCollisionTrajectory(
      trajectory, segment, 2.0, 6.0);
  if (sampled.size() < 2) return {};
  const CruisePredictedPose ego_pose{ego.x, ego.y, yaw(ego)};
  const double stop_ego_offset =
      (driving_forward ? vehicle_.front_m : vehicle_.rear_m) + 3.0;
  const double prefix_arc = arcAtIndex(trajectory, segment);
  const double path_length = arcAtIndex(
      trajectory, trajectory.size() - 1);
  const auto behavior_stop_arc = firstStopArc(trajectory, ego_index);
  const double elapsed_s = perception.stamp_s > 0.0 ?
      std::max(0.0, now_s - perception.stamp_s) : 0.0;
  std::vector<CruiseStopObservation> observed;
  std::vector<CruiseStopTrackedCandidate> found;
  std::vector<std::string> cruise_ids;
  observed.reserve(perception.objects.size());
  for (const auto &object : perception.objects) {
    observed.push_back({object.id, object.velocity_x_mps});
    const auto object_max_length = obstacleMaxLength(object);
    if (object.classification > 7 || !object_max_length) continue;
    const auto pose = currentCruiseObjectPose(
        {object.x, object.y, object.yaw_rad},
        object.predicted_paths, elapsed_s);
    const CruiseObjectKinematics motion{
        pose.x, pose.y, pose.yaw_rad,
        object.velocity_x_mps, object.velocity_y_mps};
    const auto projected = projectCruiseObjectToTrajectory(
        trajectory, ego_index, motion);
    if (!projected || !passesObstacleCruiseRoughLateralFilter(
            *projected, *object_max_length,
            vehicle_.width_m))
      continue;
    // 다각형 외곽이 기록되지 않았으면 길이·폭 직사각형으로 검사합니다.
    const CruiseObjectShape shape{
        object.shape_type == 2 && object.footprint.empty() ? 0 :
            object.shape_type,
        object.length_m, object.width_m, object.footprint};
    const double lateral = cruiseObjectLateralDistance(
        sampled, pose, shape, vehicle_.front_m, vehicle_.rear_m,
        vehicle_.width_m, ego_pose);
    CruiseCandidateDiagnostic diagnostic;
    if (isCruiseCandidate(
            object, pose, shape, *projected, sampled, ego_pose,
            lateral, ego_speed_mps, driving_forward, elapsed_s, cruise_history_,
            cruise_parameters_, vehicle_, &diagnostic)) {
      cruise_ids.push_back(object.id);
      cruise_diagnostics_.push_back(diagnostic);
      pid_obstacles_.push_back({
          prefix_arc + diagnostic.first_collision_arc_m -
              arcAtPoint(trajectory, ego.x, ego.y) - vehicle_.front_m,
          projected->longitudinal_velocity_mps});
      continue;
    }
    CruiseStopBehaviorInput decision;
    decision.supported_class = object.classification <= 7;
    decision.inside_class = true;
    decision.outside_class =
        cruise_parameters_.outside_stop_classes[object.classification];
    decision.precise_lateral_distance_m = lateral;
    decision.approach_velocity_mps = projected->approach_velocity_mps;
    CruiseStopBehaviorParameters stop_parameters;
    stop_parameters.maximum_lateral_time_s =
        cruise_parameters_.outside_stop_lateral_time_s;
    stop_parameters.collision_time_margin_s =
        cruise_parameters_.stop_collision_time_margin_s;
    std::optional<double> collision_distance;
    if (lateral > stop_parameters.maximum_lateral_margin_m) {
      // 원본 createStopObstacleForPredictedObject의 차로 밖 분기입니다.
      if (!decision.outside_class ||
          lateral / std::max(1e-6, projected->approach_velocity_mps) >
              stop_parameters.maximum_lateral_time_s)
        continue;
      double horizon_s = 10.0;
      const double object_speed = std::hypot(
          object.velocity_x_mps, object.velocity_y_mps);
      if (object.classification == 6)
        horizon_s = object_speed /
            (2.0 * cruise_parameters_.bicycle_deceleration_mps2);
      else if (object.classification == 7)
        horizon_s = object_speed /
            (2.0 * cruise_parameters_.pedestrian_deceleration_mps2);
      std::vector<const CruisePredictedPath *> ranked;
      for (const auto &path : object.predicted_paths) ranked.push_back(&path);
      std::sort(ranked.begin(), ranked.end(),
                [](const auto *a, const auto *b) {
                  return a->confidence > b->confidence;
                });
      decision.has_predicted_path = !ranked.empty();
      const std::size_t limit = std::min(
          ranked.size(), cruise_parameters_.outside_stop_predicted_path_count);
      for (std::size_t index = 0; index < limit; ++index) {
        const auto samples = resampleCruisePredictedPath(
            *ranked[index], 0.1, horizon_s);
        // 원본은 기존 주행 다각형을 그대로 검사합니다. 정지 횡여유 0.3m는
        // getCollisionPoints의 빠른 거리 필터에만 더합니다.
        const auto points = findCruiseCollisionPoints(
            sampled, samples, shape, vehicle_.front_m, vehicle_.rear_m,
            vehicle_.width_m, elapsed_s, horizon_s,
            *object_max_length + 2.0 +
                std::hypot(vehicle_.front_m + vehicle_.rear_m,
                           vehicle_.width_m * 0.5 + 0.3),
            driving_forward, ego_pose);
        if (points.empty()) continue;
        const auto first_index = points.front().trajectory_index;
        const auto &first_pose = sampled[first_index];
        const double angle = yaw(first_pose);
        const double bumper_x = first_pose.x +
            (driving_forward ? vehicle_.front_m : -vehicle_.rear_m) *
                std::cos(angle);
        const double bumper_y = first_pose.y +
            (driving_forward ? vehicle_.front_m : -vehicle_.rear_m) *
                std::sin(angle);
        double maximum_length = 0.0;
        for (const auto &point : points)
          maximum_length = std::max(maximum_length, std::abs(
              std::cos(angle) * (point.x - bumper_x) +
              std::sin(angle) * (point.y - bumper_y)));
        collision_distance = arcAtIndex(sampled, first_index) -
            maximum_length;
        const double reach_s = cruiseStopTimeToReachCollision(
            arcAtPoint(sampled, points.front().x, points.front().y) -
                arcAtPoint(sampled, ego.x, ego.y),
            stop_ego_offset, ego_speed_mps);
        decision.collision_time_gap_s = cruiseStopCollisionTimeGap(
            reach_s, stop_ego_offset, ego_speed_mps,
            perception.stamp_s + points.front().time_from_object_stamp_s -
                now_s,
            perception.stamp_s + points.back().time_from_object_stamp_s -
                now_s);
        if (decision.collision_time_gap_s <=
            stop_parameters.collision_time_margin_s) {
          decision.has_collision_point = true;
          break;
        }
      }
    } else {
      const auto collision = findCruiseCurrentStopCollision(
          sampled, pose, shape, vehicle_.front_m, vehicle_.rear_m,
          vehicle_.width_m, 0.3, driving_forward, ego_pose);
      decision.has_collision_point = collision.has_value();
      if (collision) collision_distance = collision->distance_from_start_m;
      decision.time_to_reach_stop_point_s = collision ?
          cruiseStopTimeToReachCollision(
              arcAtPoint(sampled, collision->x, collision->y) -
                  arcAtPoint(sampled, ego.x, ego.y),
              stop_ego_offset, ego_speed_mps) : 0.0;
      if (collision && decision.time_to_reach_stop_point_s >
              cruise_parameters_.stop_collision_time_margin_s &&
          !object.predicted_paths.empty()) {
      const auto best = std::max_element(
          object.predicted_paths.begin(), object.predicted_paths.end(),
          [](const auto &left, const auto &right) {
            return left.confidence < right.confidence;
          });
      const auto samples = resampleCruisePredictedPath(
          *best, 0.1, 10.0);
      if (!samples.empty()) {
        decision.checked_future_collision = true;
        decision.has_future_collision =
            findCruiseCurrentStopCollision(
                sampled,
                futurePose(samples,
                           decision.time_to_reach_stop_point_s -
                               cruise_parameters_.stop_collision_time_margin_s),
                shape, vehicle_.front_m, vehicle_.rear_m,
                vehicle_.width_m,
                0.3, driving_forward, ego_pose).has_value();
      }
      }
    }
    if (classifyCruiseStopBehavior(decision, stop_parameters) !=
        CruiseStopRejection::None)
      continue;
    found.push_back({object.id, object.classification,
                     perception.stamp_s > 0.0 ? perception.stamp_s : now_s,
                     prefix_arc + *collision_distance});
  }
  cruise_history_.replacePreviousCruiseIds(cruise_ids);
  // 원본 checkConsistency는 trajectory callback 시각이 아니라
  // PredictedObjects 헤더 시각을 이전 객체 stamp와 비교한다.
  const double object_stamp_s = perception.stamp_s > 0.0 ?
      perception.stamp_s : now_s;
  const auto held = consistency_.update(
      object_stamp_s, observed, found, 1.0,
      cruise_parameters_.inside_cruise_entry_speed_mps);
  claimed_object_ids_ = std::move(cruise_ids);
  for (const auto &item : held) {
    if (std::find(claimed_object_ids_.begin(),
                  claimed_object_ids_.end(), item.id) ==
        claimed_object_ids_.end())
      claimed_object_ids_.push_back(item.id);
  }
  std::vector<ObstacleCruiseStopCandidate> candidates;
  candidates.reserve(held.size());
  for (const auto &item : held) {
    ObstacleCruiseStopCandidate candidate;
    candidate.classification = item.classification;
    candidate.collision_distance_from_start =
        item.collision_distance_from_start_m;
    candidate.desired_margin = chooseCruiseStopMargin(
        candidate.collision_distance_from_start,
        path_length, behavior_stop_arc,
        cruise_parameters_.stop_safe_distance_margin_m, 3.0, 3.0);
    candidate.type_specific_parameters = item.classification == 0;
    candidate.sudden_acceleration_limit = -1.2;
    candidate.sudden_object_acceleration_threshold = -1.1;
    candidates.push_back(candidate);
  }
  return candidates;
}

const std::vector<std::string> &
PlanningCruiseStopCandidateCore::claimedObjectIds() const {
  return claimed_object_ids_;
}

const std::vector<CruisePidObstacle> &
PlanningCruiseStopCandidateCore::pidObstacles() const {
  return pid_obstacles_;
}

const std::vector<CruiseCandidateDiagnostic> &
PlanningCruiseStopCandidateCore::cruiseDiagnostics() const {
  return cruise_diagnostics_;
}

void PlanningCruiseStopCandidateCore::reset() {
  consistency_.reset();
  cruise_history_.reset();
  claimed_object_ids_.clear();
  pid_obstacles_.clear();
  cruise_diagnostics_.clear();
}

}  // planning 네임스페이스
