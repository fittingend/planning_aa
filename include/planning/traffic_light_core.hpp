// Copyright 2020 Tier IV, Inc.
// SPDX-License-Identifier: Apache-2.0
// 신호등 정지선의 속도 계획과 상태 전환을 위한 독립 계산 코어입니다.

#pragma once

#include "planning/stop_line_core.hpp"
#include "planning/traffic_light_signal.hpp"

#include <optional>

namespace planning {

enum class TrafficLightState { Approach, GoOut };

struct TrafficLightParameters {
  double stop_margin_m = 0.0;
  double front_offset_m = 3.79;
  double line_extension_m = 5.0;
  double signal_timeout_s = 1.0;
  double stop_hysteresis_s = 0.1;
  double yellow_period_s = 2.75;
  double max_stop_acceleration_mps2 = -2.8;
  double max_stop_jerk_mps3 = -5.0;
  double response_delay_s = 0.5;
  bool enable_pass_judge = true;
  bool enable_rtc = false;
  bool simulation = false;
};

struct TrafficLightInput {
  double ego_x = 0.0;
  double ego_y = 0.0;
  double ego_velocity_mps = 0.0;
  double ego_acceleration_mps2 = 0.0;
  double now_s = 0.0;
  std::optional<double> signal_stamp_s;
  std::optional<bool> signal_requires_stop;
  bool rtc_activated = false;
};

struct TrafficLightResult {
  Trajectory trajectory;
  std::optional<double> stop_arc_m;
  double distance_to_stop_m = 0.0;
  TrafficLightState state = TrafficLightState::Approach;
  bool safe = true;
  bool activated = true;
  bool applied_stop = false;
};

class TrafficLightCore {
 public:
  TrafficLightCore(StopLineSegment line, TrafficLightParameters parameters = {});

  TrafficLightResult apply(const Trajectory &path, const TrafficLightInput &input);
  TrafficLightResult apply(
      const Trajectory &path, TrafficLightInput input,
      const TrafficLightSignalObservation &observation);
  TrafficLightState state() const noexcept { return state_; }

 private:
  StopLineSegment line_;
  TrafficLightParameters parameters_;
  TrafficLightState state_ = TrafficLightState::Approach;
  bool previous_stop_ = false;
  bool safe_ = false;
  bool activated_ = false;
  std::optional<double> stop_signal_since_s_;
  std::optional<double> last_signal_stamp_s_;
  bool last_signal_requires_stop_ = false;
};

}  // planning 네임스페이스
