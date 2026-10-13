// Copyright 2020 Tier IV, Inc.
// SPDX-License-Identifier: Apache-2.0
// 원본 scenario_selector/node.cpp의 상태 전환과 출력 판정을 이관했습니다.

#include "planning/scenario_selector.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace planning {

ScenarioSelectorCore::ScenarioSelectorCore(ScenarioSelectorParameters parameters)
    : parameters_(parameters) {
  if (!(parameters_.max_message_delay_s >= 0.0) ||
      !(parameters_.arrived_distance_m >= 0.0) ||
      !(parameters_.stopped_time_s > 0.0) ||
      !(parameters_.stopped_velocity_mps >= 0.0) ||
      !(parameters_.lane_stopping_timeout_s >= 0.0) ||
      !(parameters_.empty_parking_timeout_s >= 0.0)) {
    throw std::invalid_argument("시나리오 선택기 설정이 잘못되었습니다");
  }
}

void ScenarioSelectorCore::onMap(bool ready) {
  map_ready_ = ready;
  // 원본은 지도를 받을 때 경로 처리기를 새로 만들므로 이전 경로 ID를 잊습니다.
  applied_route_id_.clear();
}

void ScenarioSelectorCore::onRoute(const std::string &route_id) {
  if (!map_ready_ || applied_route_id_ != route_id) {
    current_ = Scenario::Empty;
  }
  route_received_ = true;
  applied_route_id_ = route_id;
}

void ScenarioSelectorCore::onOdometry(
    double stamp_s, double x, double y, double velocity_mps) {
  if (!std::isfinite(stamp_s) || !std::isfinite(x) || !std::isfinite(y) ||
      !std::isfinite(velocity_mps)) {
    throw std::invalid_argument("차량 상태에는 유한한 값이 필요합니다");
  }
  ego_x_ = x;
  ego_y_ = y;
  odometry_received_ = true;
  speeds_.push_back({stamp_s, velocity_mps});
  // 원본은 수신 시각이 아닌 오도메트리 타임스탬프로 정차 이력을 자릅니다.
  while (speeds_.size() > 1 &&
         stamp_s - speeds_.front().stamp_s >= parameters_.stopped_time_s) {
    speeds_.pop_front();
  }
}

void ScenarioSelectorCore::onOperationMode(bool autonomous, bool control_enabled) {
  autonomous_ = autonomous;
  control_enabled_ = control_enabled;
  operation_mode_received_ = true;
}

void ScenarioSelectorCore::onParkingCompleted(bool completed) {
  parking_completed_ = completed;
}

std::optional<ScenarioTrajectory> ScenarioSelectorCore::publishIfCurrent(
    const ScenarioTrajectory &trajectory, double now_s, Scenario expected) const {
  if (current_ != expected || !std::isfinite(now_s) ||
      !std::isfinite(trajectory.stamp_s) ||
      now_s - trajectory.stamp_s > parameters_.max_message_delay_s) {
    return std::nullopt;
  }
  return trajectory;
}

std::optional<ScenarioTrajectory> ScenarioSelectorCore::onLaneTrajectory(
    ScenarioTrajectory trajectory, double now_s) {
  lane_trajectory_ = std::move(trajectory);
  return publishIfCurrent(*lane_trajectory_, now_s, Scenario::LaneDriving);
}

std::optional<ScenarioTrajectory> ScenarioSelectorCore::onParkingTrajectory(
    ScenarioTrajectory trajectory, double now_s) {
  parking_trajectory_ = std::move(trajectory);
  return publishIfCurrent(*parking_trajectory_, now_s, Scenario::Parking);
}

bool ScenarioSelectorCore::stopped() const {
  for (const auto &sample : speeds_) {
    if (std::abs(sample.velocity_mps) > parameters_.stopped_velocity_mps) return false;
  }
  return true;
}

bool ScenarioSelectorCore::nearEnd() const {
  const auto &trajectory = current_ == Scenario::Parking ? parking_trajectory_
                                                          : lane_trajectory_;
  if (!trajectory || trajectory->points.empty()) return false;
  const auto &last = trajectory->points.back();
  return std::hypot(ego_x_ - last.x, ego_y_ - last.y) <
         parameters_.arrived_distance_m;
}

Scenario ScenarioSelectorCore::selectByPosition(const ScenarioPosition &position) {
  if (current_ == Scenario::Empty) {
    if (position.in_lane && position.goal_in_lane) return Scenario::LaneDriving;
    if (position.in_parking_lot) return Scenario::Parking;
    return Scenario::LaneDriving;
  }
  if (current_ == Scenario::LaneDriving && position.in_parking_lot &&
      !position.goal_in_lane) {
    return Scenario::Parking;
  }
  if (current_ == Scenario::Parking && parking_completed_ && position.in_lane) {
    parking_completed_ = false;
    return Scenario::LaneDriving;
  }
  return current_;
}

bool ScenarioSelectorCore::switchToParking(
    double now_s, const ScenarioPosition &position) {
  if (!stopped() || !autonomous_ || !control_enabled_ ||
      !position.in_parking_lot || position.goal_in_lane) {
    lane_stop_since_s_.reset();
    return false;
  }
  if (!lane_stop_since_s_) {
    lane_stop_since_s_ = now_s;
    return false;
  }
  return now_s - *lane_stop_since_s_ > parameters_.lane_stopping_timeout_s;
}

bool ScenarioSelectorCore::switchToLane(
    double now_s, const ScenarioPosition &position) {
  if (!parking_trajectory_ || parking_trajectory_->points.size() > 1 ||
      !position.along_lane) {
    empty_parking_since_s_.reset();
    return false;
  }
  if (!empty_parking_since_s_) {
    empty_parking_since_s_ = now_s;
    return false;
  }
  return now_s - *empty_parking_since_s_ > parameters_.empty_parking_timeout_s;
}

std::optional<ScenarioSelection> ScenarioSelectorCore::tick(
    double now_s, const ScenarioPosition &position, bool route_ready) {
  if (!std::isfinite(now_s) || !odometry_received_ || !map_ready_ ||
      !route_received_ || !operation_mode_received_ || !route_ready) {
    return std::nullopt;
  }
  if (current_ == Scenario::Empty) current_ = selectByPosition(position);
  const Scenario previous = current_;
  if (nearEnd() && stopped()) current_ = selectByPosition(position);
  if (parameters_.enable_mode_switching) {
    if (current_ == Scenario::LaneDriving && switchToParking(now_s, position)) {
      current_ = Scenario::Parking;
    } else if (current_ == Scenario::Parking && switchToLane(now_s, position)) {
      current_ = Scenario::LaneDriving;
    }
  }
  if (current_ != previous) {
    lane_stop_since_s_.reset();
    empty_parking_since_s_.reset();
  }
  return ScenarioSelection{
      current_, current_ == Scenario::Parking, current_ != previous};
}

}  // planning 네임스페이스
