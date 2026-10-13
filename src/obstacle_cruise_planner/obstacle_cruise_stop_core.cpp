// SPDX-License-Identifier: Apache-2.0
// 원본 PlannerInterface::generateStopTrajectory의 거리·유지 계산입니다.

#include "planning/obstacle_cruise_stop_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace planning {
namespace {
double distanceToStop(double velocity, double positive_acceleration,
                      double negative_acceleration) {
  return velocity < 0.0 ?
      -velocity * velocity / (2.0 * positive_acceleration) :
      -velocity * velocity / (2.0 * negative_acceleration);
}

double previousFrontOffset(const Trajectory &previous,
                           const TrajectoryPoint &current_front) {
  double accumulated = 0.0;
  double best = std::numeric_limits<double>::infinity();
  double offset = 0.0;
  for (std::size_t i = 0; i + 1 < previous.size(); ++i) {
    const double dx = previous[i + 1].x - previous[i].x;
    const double dy = previous[i + 1].y - previous[i].y;
    const double length = std::hypot(dx, dy);
    if (length < 1.0e-9) continue;
    const double ratio = std::clamp(
        ((current_front.x - previous[i].x) * dx +
         (current_front.y - previous[i].y) * dy) / (length * length),
        0.0, 1.0);
    const double distance = std::hypot(
        previous[i].x + ratio * dx - current_front.x,
        previous[i].y + ratio * dy - current_front.y);
    if (distance < best) { best = distance; offset = accumulated + ratio * length; }
    accumulated += length;
  }
  return offset;
}

std::optional<std::size_t> insertStop(Trajectory &trajectory, double distance) {
  if (trajectory.size() < 2 || distance < 0.0) return std::nullopt;
  double accumulated = 0.0;
  for (std::size_t i = 0; i + 1 < trajectory.size(); ++i) {
    const auto &front = trajectory[i];
    const auto &back = trajectory[i + 1];
    const double segment = std::hypot(back.x - front.x, back.y - front.y);
    if (distance > accumulated + segment) { accumulated += segment; continue; }
    const double ratio = segment < 1.0e-9 ? 0.0 :
        std::clamp((distance - accumulated) / segment, 0.0, 1.0);
    const double x = front.x + ratio * (back.x - front.x);
    const double y = front.y + ratio * (back.y - front.y);
    std::size_t index;
    if (std::hypot(x - front.x, y - front.y) < 1.0e-3) {
      index = i;
    } else if (std::hypot(x - back.x, y - back.y) < 1.0e-3) {
      index = i + 1;
    } else {
      auto point = front;
      point.x = x;
      point.y = y;
      point.z = front.z + ratio * (back.z - front.z);
      const double yaw = std::atan2(back.y - y, back.x - x);
      point.orientation_z = std::sin(yaw / 2.0);
      point.orientation_w = std::cos(yaw / 2.0);
      trajectory[i].orientation_z = point.orientation_z;
      trajectory[i].orientation_w = point.orientation_w;
      trajectory.insert(trajectory.begin() + i + 1, point);
      index = i + 1;
    }
    for (std::size_t j = index; j < trajectory.size(); ++j)
      trajectory[j].longitudinal_velocity_mps = 0.0F;
    return index;
  }
  return std::nullopt;
}
}  // 익명 네임스페이스

CruiseStopRejection classifyCruiseStopBehavior(
    const CruiseStopBehaviorInput &input,
    const CruiseStopBehaviorParameters &p) {
  const double values[] = {
      input.precise_lateral_distance_m, input.approach_velocity_mps,
      input.collision_time_gap_s, input.time_to_reach_stop_point_s,
      p.maximum_lateral_margin_m, p.maximum_lateral_time_s,
      p.collision_time_margin_s};
  if (!std::all_of(std::begin(values), std::end(values),
                   [](double value) { return std::isfinite(value); }) ||
      input.precise_lateral_distance_m < 0.0 ||
      p.maximum_lateral_margin_m < 0.0 ||
      p.maximum_lateral_time_s < 0.0 || p.collision_time_margin_s < 0.0)
    throw std::invalid_argument("순항 정지 객체 판정 입력이 유효하지 않습니다");
  if (!input.supported_class) return CruiseStopRejection::UnsupportedClass;
  if (input.precise_lateral_distance_m >
      std::max(p.maximum_lateral_margin_m, 1e-3)) {
    if (!input.outside_class)
      return CruiseStopRejection::OutsideUnsupportedClass;
    if (input.precise_lateral_distance_m /
            std::max(1e-6, input.approach_velocity_mps) >
        p.maximum_lateral_time_s)
      return CruiseStopRejection::OutsideTooFar;
    if (!input.has_predicted_path)
      return CruiseStopRejection::NoPredictedPath;
    if (!input.has_collision_point)
      return CruiseStopRejection::NoCollision;
    if (p.collision_time_margin_s < input.collision_time_gap_s)
      return CruiseStopRejection::OutsideTimeGap;
  } else {
    if (!input.inside_class)
      return CruiseStopRejection::InsideUnsupportedClass;
    if (!input.has_collision_point)
      return CruiseStopRejection::NoCollision;
    if (input.time_to_reach_stop_point_s > p.collision_time_margin_s &&
        input.checked_future_collision && !input.has_future_collision)
      return CruiseStopRejection::TransientInsideObject;
  }
  return CruiseStopRejection::None;
}

double cruiseStopTimeToReachCollision(
    const double signed_distance_m, const double ego_offset_m,
    const double ego_speed_mps, const double minimum_speed_mps) {
  if (!std::isfinite(signed_distance_m) || !std::isfinite(ego_offset_m) ||
      !std::isfinite(ego_speed_mps) || !std::isfinite(minimum_speed_mps) ||
      ego_offset_m < 0.0 || minimum_speed_mps <= 0.0)
    throw std::invalid_argument("순항 정지 도착 시간 입력이 유효하지 않습니다");
  return (std::abs(signed_distance_m) - ego_offset_m) /
         std::max(minimum_speed_mps, std::abs(ego_speed_mps));
}

double cruiseStopCollisionTimeGap(
    const double ego_time_to_reach_s, const double ego_offset_m,
    const double ego_speed_mps, const double object_start_s,
    const double object_end_s, const double minimum_speed_mps) {
  const double values[] = {
      ego_time_to_reach_s, ego_offset_m, ego_speed_mps,
      object_start_s, object_end_s, minimum_speed_mps};
  if (!std::all_of(std::begin(values), std::end(values),
                   [](double value) { return std::isfinite(value); }) ||
      ego_offset_m < 0.0 || minimum_speed_mps <= 0.0 ||
      object_start_s > object_end_s)
    throw std::invalid_argument("순항 정지 충돌 시간 입력이 유효하지 않습니다");
  const double ego_leave = ego_time_to_reach_s + ego_offset_m /
      std::max(minimum_speed_mps, ego_speed_mps);
  if (ego_leave < object_start_s)
    return object_start_s - ego_time_to_reach_s;
  if (object_end_s < ego_time_to_reach_s)
    return ego_time_to_reach_s - object_end_s;
  return 0.0;
}

double chooseCruiseStopMargin(
    const double collision_arc_m, const double trajectory_length_m,
    const std::optional<double> behavior_stop_arc_m,
    const double safe_margin_m, const double terminal_margin_m,
    const double minimum_behavior_margin_m) {
  if (!std::isfinite(collision_arc_m) ||
      !std::isfinite(trajectory_length_m) ||
      !std::isfinite(safe_margin_m) ||
      !std::isfinite(terminal_margin_m) ||
      !std::isfinite(minimum_behavior_margin_m) ||
      (behavior_stop_arc_m && !std::isfinite(*behavior_stop_arc_m)) ||
      trajectory_length_m < 0.0 || safe_margin_m < 0.0 ||
      terminal_margin_m < 0.0 || minimum_behavior_margin_m < 0.0)
    throw std::invalid_argument("순항 정지 여유 입력이 유효하지 않습니다");
  if (collision_arc_m > trajectory_length_m) return terminal_margin_m;
  if (behavior_stop_arc_m) {
    const double difference = *behavior_stop_arc_m -
                              (collision_arc_m - safe_margin_m);
    if (difference > 0.0 && difference < safe_margin_m)
      return minimum_behavior_margin_m;
  }
  return safe_margin_m;
}

std::vector<CruiseStopTrackedCandidate> CruiseStopConsistency::update(
    const double current_time_s,
    const std::vector<CruiseStopObservation> &observed,
    const std::vector<CruiseStopTrackedCandidate> &current,
    const double hold_time_s, const double maximum_velocity_mps) {
  if (!std::isfinite(current_time_s) || hold_time_s < 0.0 ||
      maximum_velocity_mps < 0.0 || !std::isfinite(hold_time_s) ||
      !std::isfinite(maximum_velocity_mps))
    throw std::invalid_argument("순항 정지 객체 유지 입력이 유효하지 않습니다");
  auto augmented = current;
  for (const auto &previous : previous_closest_) {
    const auto observation = std::find_if(
        observed.begin(), observed.end(), [&](const auto &item) {
          return item.id == previous.id;
        });
    if (observation == observed.end()) continue;
    const bool present = std::any_of(
        augmented.begin(), augmented.end(), [&](const auto &item) {
          return item.id == previous.id;
        });
    if (!present &&
        observation->local_velocity_x_mps < maximum_velocity_mps &&
        current_time_s - previous.object_stamp_s < hold_time_s)
      augmented.push_back(previous);
  }
  previous_closest_.clear();
  for (const auto &item : augmented) {
    const auto found = std::find_if(
        previous_closest_.begin(), previous_closest_.end(),
        [&](const auto &candidate) {
          return candidate.classification == item.classification;
        });
    if (found == previous_closest_.end()) previous_closest_.push_back(item);
    else if (item.collision_distance_from_start_m <
             found->collision_distance_from_start_m) *found = item;
  }
  return augmented;
}

void CruiseStopConsistency::reset() { previous_closest_.clear(); }

ObstacleCruiseStopCore::ObstacleCruiseStopCore(
    ObstacleCruiseStopParameters parameters) : parameters_(parameters) {}

void ObstacleCruiseStopCore::reset() {
  previous_stop_distance_.reset();
  previous_trajectory_.clear();
}

ObstacleCruiseStopResult ObstacleCruiseStopCore::plan(
    const Trajectory &trajectory, const double ego_distance_from_start,
    const double ego_velocity,
    const std::vector<ObstacleCruiseStopCandidate> &obstacles) {
  if (trajectory.size() < 2 || !std::isfinite(ego_distance_from_start) ||
      !std::isfinite(ego_velocity) ||
      parameters_.driving_acceleration_limit <= 0.0 ||
      parameters_.braking_acceleration_limit >= 0.0) {
    throw std::invalid_argument("장애물 정지 계획 입력이 유효하지 않습니다");
  }
  ObstacleCruiseStopResult result;
  result.trajectory = trajectory;
  std::vector<ObstacleCruiseStopCandidate> closest;
  for (const auto &obstacle : obstacles) {
    if (!std::isfinite(obstacle.collision_distance_from_start) ||
        !std::isfinite(obstacle.desired_margin) ||
        !std::isfinite(obstacle.sudden_acceleration_limit) ||
        !std::isfinite(obstacle.sudden_object_acceleration_threshold) ||
        !std::isfinite(obstacle.sudden_object_distance_threshold) ||
        obstacle.sudden_acceleration_limit >= 0.0 ||
        obstacle.sudden_object_acceleration_threshold >= 0.0 ||
        obstacle.sudden_object_distance_threshold < 0.0) {
      throw std::invalid_argument("장애물 정지 후보 값이 유효하지 않습니다");
    }
    const auto found = std::find_if(closest.begin(), closest.end(),
        [&](const auto &candidate) {
          return candidate.classification == obstacle.classification;
        });
    if (found == closest.end()) closest.push_back(obstacle);
    else if (obstacle.collision_distance_from_start <
             found->collision_distance_from_start) *found = obstacle;
  }
  double chosen_collision = std::numeric_limits<double>::infinity();
  for (const auto &obstacle : closest) {
    double stop = std::max(0.0, obstacle.collision_distance_from_start -
                               obstacle.desired_margin);
    if (parameters_.suppress_sudden_stop) {
      double limit = parameters_.braking_acceleration_limit;
      if (obstacle.type_specific_parameters) {
        const double threshold = std::min(
            distanceToStop(
                ego_velocity, parameters_.driving_acceleration_limit,
                obstacle.sudden_object_acceleration_threshold),
            obstacle.sudden_object_distance_threshold);
        if (stop <= threshold) {
          if (obstacle.abandon_sudden_stop) continue;
          limit = obstacle.sudden_acceleration_limit;
        }
      }
      stop = std::max(stop, ego_distance_from_start + distanceToStop(
          ego_velocity, parameters_.driving_acceleration_limit, limit));
    }
    if (result.stop_distance_from_start &&
        ((result.classification == obstacle.classification &&
          obstacle.collision_distance_from_start > chosen_collision) ||
         (result.classification != obstacle.classification &&
          stop > *result.stop_distance_from_start))) continue;
    result.stop_distance_from_start = stop;
    result.classification = obstacle.classification;
    chosen_collision = obstacle.collision_distance_from_start;
  }
  if (!result.stop_distance_from_start) {
    reset();
    return result;
  }
  if (std::abs(ego_velocity) < parameters_.stop_hold_velocity &&
      previous_stop_distance_ && previous_trajectory_.size() >= 2) {
    const double previous = *previous_stop_distance_ -
        previousFrontOffset(previous_trajectory_, trajectory.front());
    if (std::abs(previous - *result.stop_distance_from_start) <
        parameters_.stop_hold_distance) result.stop_distance_from_start = previous;
  }
  result.stop_index = insertStop(result.trajectory, *result.stop_distance_from_start);
  if (result.stop_index) {
    previous_stop_distance_ = result.stop_distance_from_start;
    previous_trajectory_ = result.trajectory;
  }
  return result;
}

}  // planning 네임스페이스
