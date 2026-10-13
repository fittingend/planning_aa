// Copyright 2020 Tier IV, Inc.
// SPDX-License-Identifier: Apache-2.0
// 시나리오 선택기의 ROS와 지도 라이브러리에 의존하지 않는 상태 전환입니다.

#pragma once

#include "planning/trajectory.hpp"

#include <deque>
#include <optional>
#include <string>

namespace planning {

enum class Scenario { Empty, LaneDriving, Parking };

struct ScenarioSelectorParameters {
  double max_message_delay_s = 1.0;
  double arrived_distance_m = 1.0;
  double stopped_time_s = 1.0;
  double stopped_velocity_mps = 0.01;
  double lane_stopping_timeout_s = 5.0;
  double empty_parking_timeout_s = 3.0;
  bool enable_mode_switching = true;
};

// 지도 및 경로 어댑터가 원본 Lanelet2 판정으로 채워야 하는 입력입니다.
struct ScenarioPosition {
  bool in_lane = false;
  bool goal_in_lane = false;
  bool in_parking_lot = false;
  bool along_lane = false;
};

struct ScenarioSelection {
  Scenario current = Scenario::Empty;
  bool parking_active = false;
  bool changed = false;
};

struct ScenarioTrajectory {
  Trajectory points;
  double stamp_s = 0.0;
};

class ScenarioSelectorCore {
 public:
  explicit ScenarioSelectorCore(ScenarioSelectorParameters parameters = {});

  void onMap(bool ready);
  void onRoute(const std::string &route_id);
  void onOdometry(double stamp_s, double x, double y, double velocity_mps);
  void onOperationMode(bool autonomous, bool control_enabled);
  void onParkingCompleted(bool completed);
  std::optional<ScenarioTrajectory> onLaneTrajectory(
      ScenarioTrajectory trajectory, double now_s);
  std::optional<ScenarioTrajectory> onParkingTrajectory(
      ScenarioTrajectory trajectory, double now_s);
  std::optional<ScenarioSelection> tick(
      double now_s, const ScenarioPosition &position, bool route_ready = true);

  Scenario currentScenario() const noexcept { return current_; }

 private:
  struct SpeedSample {
    double stamp_s;
    double velocity_mps;
  };

  Scenario selectByPosition(const ScenarioPosition &position);
  bool stopped() const;
  bool nearEnd() const;
  bool switchToParking(double now_s, const ScenarioPosition &position);
  bool switchToLane(double now_s, const ScenarioPosition &position);
  std::optional<ScenarioTrajectory> publishIfCurrent(
      const ScenarioTrajectory &trajectory, double now_s, Scenario expected) const;

  ScenarioSelectorParameters parameters_;
  Scenario current_ = Scenario::Empty;
  bool map_ready_ = false;
  bool route_received_ = false;
  bool odometry_received_ = false;
  bool operation_mode_received_ = false;
  bool autonomous_ = false;
  bool control_enabled_ = false;
  bool parking_completed_ = false;
  double ego_x_ = 0.0;
  double ego_y_ = 0.0;
  std::string applied_route_id_;
  std::deque<SpeedSample> speeds_;
  std::optional<double> lane_stop_since_s_;
  std::optional<double> empty_parking_since_s_;
  std::optional<ScenarioTrajectory> lane_trajectory_;
  std::optional<ScenarioTrajectory> parking_trajectory_;
};

}  // planning 네임스페이스
