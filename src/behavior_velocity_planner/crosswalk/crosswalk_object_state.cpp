// Copyright 2020 Tier IV, Inc.
// SPDX-License-Identifier: Apache-2.0
// 원본 횡단보도 객체 관리자의 프레임별 상태 전환을 계산합니다.

#include "planning/crosswalk_object_state.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace planning {
namespace {

double interpolateTimeout(
    const std::vector<double> &distance,
    const std::vector<double> &timeout, double query) {
  if (query <= distance.front()) return timeout.front();
  if (query >= distance.back()) return timeout.back();
  for (std::size_t i = 1; i < distance.size(); ++i) {
    if (query <= distance[i]) {
      const double ratio = (query - distance[i - 1]) /
                           (distance[i] - distance[i - 1]);
      return timeout[i - 1] + ratio * (timeout[i] - timeout[i - 1]);
    }
  }
  return timeout.back();
}

}  // 익명 네임스페이스

bool pedestriansHaveRedSignal(
    const std::vector<CrosswalkPedestrianSignal> &signals,
    double now_s, double timeout_s) {
  if (!std::isfinite(now_s) || !std::isfinite(timeout_s) || timeout_s < 0.0) {
    throw std::invalid_argument("횡단보도 신호 시각이 잘못되었습니다");
  }
  for (const auto &signal : signals) {
    if (!std::isfinite(signal.stamp_s) ||
        now_s - signal.stamp_s > timeout_s) continue;
    for (const auto &element : signal.elements) {
      if (element.color == TrafficLampColor::Red &&
          element.shape == TrafficLampShape::Circle) return true;
    }
  }
  return false;
}

CrosswalkObjectStateTracker::CrosswalkObjectStateTracker(
    CrosswalkObjectStateParameters parameters)
    : parameters_(std::move(parameters)) {
  const auto &distance = parameters_.distance_to_crosswalk_m;
  const auto &timeout = parameters_.no_intention_timeout_s;
  if (!std::isfinite(parameters_.stopped_speed_mps) ||
      !std::isfinite(parameters_.vehicle_cross_angle_threshold_rad) ||
      parameters_.stopped_speed_mps < 0.0 ||
      parameters_.vehicle_cross_angle_threshold_rad < 0.0 ||
      distance.empty() || distance.size() != timeout.size()) {
    throw std::invalid_argument("횡단보도 객체 상태 설정이 잘못되었습니다");
  }
  for (std::size_t i = 0; i < distance.size(); ++i) {
    if (!std::isfinite(distance[i]) || !std::isfinite(timeout[i]) ||
        timeout[i] < 0.0 || (i && distance[i] <= distance[i - 1])) {
      throw std::invalid_argument("횡단보도 객체 의도 시간 표가 잘못되었습니다");
    }
  }
}

void CrosswalkObjectStateTracker::beginFrame() {
  seen_.clear();
  ignore_crosswalk_ = false;
}

void CrosswalkObjectStateTracker::beginFrame(
    const std::vector<CrosswalkPedestrianSignal> &signals,
    double now_s, double timeout_s) {
  beginFrame();
  ignore_crosswalk_ = pedestriansHaveRedSignal(signals, now_s, timeout_s);
}

CrosswalkCollisionState CrosswalkObjectStateTracker::update(
    const std::string &id, const CrosswalkObjectObservation &observation) {
  if (id.empty() || !std::isfinite(observation.now_s) ||
      !std::isfinite(observation.speed_mps) ||
      !std::isfinite(observation.distance_to_crosswalk_m) ||
      observation.speed_mps < 0.0) {
    throw std::invalid_argument("횡단보도 객체 관측값이 잘못되었습니다");
  }
  if (ignore_crosswalk_) return CrosswalkCollisionState::Ignore;
  seen_.insert(id);
  const auto initial = observation.crosswalk_has_traffic_light &&
      parameters_.ignore_new_away_object_at_traffic_light &&
      observation.away_from_attention_area
      ? CrosswalkCollisionState::Ignore : CrosswalkCollisionState::Yield;
  auto [iterator, inserted] = objects_.try_emplace(id, ObjectState{initial, std::nullopt});
  (void)inserted;
  auto &object = iterator->second;
  const bool stopped = observation.speed_mps < parameters_.stopped_speed_mps;
  if (stopped && observation.ego_yielding) {
    if (!object.stopped_since_s) object.stopped_since_s = observation.now_s;
    const double timeout = interpolateTimeout(
        parameters_.distance_to_crosswalk_m,
        parameters_.no_intention_timeout_s,
        observation.distance_to_crosswalk_m);
    if (observation.now_s - *object.stopped_since_s >= timeout &&
        observation.away_from_attention_area) {
      object.collision_state = CrosswalkCollisionState::Ignore;
      return object.collision_state;
    }
  } else {
    object.stopped_since_s.reset();
  }
  if (stopped && object.collision_state == CrosswalkCollisionState::Ignore) {
    return object.collision_state;
  }
  if (!observation.collision_times_s) return object.collision_state;
  if (observation.bicycle_or_motorcycle &&
      observation.ego_passage_direction_rad &&
      observation.object_passage_direction_rad) {
    constexpr double half_pi = 1.57079632679489661923;
    const double difference = std::abs(std::fmod(
        *observation.object_passage_direction_rad -
        *observation.ego_passage_direction_rad, half_pi));
    if (std::min(difference, half_pi - difference) <
        parameters_.vehicle_cross_angle_threshold_rad) {
      object.collision_state = CrosswalkCollisionState::Ignore;
      return object.collision_state;
    }
  }
  object.collision_state = classifyCrosswalkPass(
      observation.collision_times_s->first,
      observation.collision_times_s->second,
      object.collision_state, parameters_.pass);
  return object.collision_state;
}

void CrosswalkObjectStateTracker::endFrame() {
  for (auto iterator = objects_.begin(); iterator != objects_.end();) {
    if (seen_.count(iterator->first) == 0) {
      iterator = objects_.erase(iterator);
    } else {
      ++iterator;
    }
  }
}

std::optional<CrosswalkCollisionState> CrosswalkObjectStateTracker::state(
    const std::string &id) const {
  const auto iterator = objects_.find(id);
  if (iterator == objects_.end()) return std::nullopt;
  return iterator->second.collision_state;
}

}  // planning 네임스페이스
