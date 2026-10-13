// Copyright 2022 Tier IV, Inc.
// SPDX-License-Identifier: Apache-2.0
// 가상 신호등의 승인·완료 상태에 따른 정지 대상 선택입니다.

#pragma once

#include "planning/trajectory_stop.hpp"
#include "planning/stop_line_core.hpp"

#include <optional>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace planning {

enum class VirtualTrafficLightState {
  None, Requesting, Passing, Finalizing, Finalized
};

enum class VirtualTrafficLightStop { None, StopLine, EndLine };

struct VirtualTrafficLightParameters {
  double maximum_message_delay_s = 1.0;
  bool check_timeout_after_stop_line = false;
};

struct VirtualTrafficLightMessage {
  double stamp_s = 0.0;
  bool approved = false;
  bool finalized = false;
};

struct VirtualTrafficLightInput {
  double now_s = 0.0;
  bool has_end_line = true;
  bool before_start_line = false;
  bool after_end_line = false;
  bool has_stop_line = true;
  bool before_stop_line = true;
  bool near_end_line = false;
  bool vehicle_stopped = false;
  std::optional<VirtualTrafficLightMessage> message;
};

struct VirtualTrafficLightResult {
  VirtualTrafficLightState state = VirtualTrafficLightState::None;
  VirtualTrafficLightStop stop = VirtualTrafficLightStop::None;
};

struct VirtualTrafficLightCommandMetadata {
  std::string instrument_type;
  std::string instrument_id;
  std::vector<std::pair<std::string, std::string>> custom_tags;
};

struct VirtualTrafficLightCommand {
  double stamp_s = 0.0;
  std::uint8_t state = 0;
  VirtualTrafficLightCommandMetadata metadata;
};

// 종료선이 있는 모듈에서 원본의 인프라 명령 필드를 구성합니다.
std::optional<VirtualTrafficLightCommand> makeVirtualTrafficLightCommand(
    const VirtualTrafficLightInput &input,
    const VirtualTrafficLightResult &decision,
    const VirtualTrafficLightCommandMetadata &metadata);

class VirtualTrafficLightCore {
 public:
  explicit VirtualTrafficLightCore(VirtualTrafficLightParameters parameters = {});
  VirtualTrafficLightResult apply(const VirtualTrafficLightInput &input);
  VirtualTrafficLightState state() const noexcept { return state_; }

 private:
  VirtualTrafficLightParameters parameters_;
  VirtualTrafficLightState state_ = VirtualTrafficLightState::None;
};

struct VirtualTrafficLightPathInput {
  Trajectory trajectory;
  VirtualTrafficLightResult decision;
  std::optional<double> stop_line_arc_m;
  std::optional<double> end_line_arc_m;
  double ego_arc_m = 0.0;
  double front_offset_m = 0.0;
  double hold_stop_margin_m = 0.0;
  bool vehicle_stopped = false;
  bool before_stop_line = true;
};

// 정지선 교차점의 경로 호길이는 지도 연결 계층에서 전달합니다.
TrajectoryStopResult applyVirtualTrafficLightStopVelocity(
    const VirtualTrafficLightPathInput &input);

struct VirtualTrafficLightLineArcs {
  std::optional<double> start_arc_m;
  std::optional<double> stop_arc_m;
  std::optional<double> end_arc_m;
};

// 첫 종료선보다 앞에 있는 마지막 시작선·정지선 교차점을 찾습니다.
VirtualTrafficLightLineArcs findVirtualTrafficLightLineArcs(
    const Trajectory &trajectory,
    const std::vector<StopLineSegment> &start_lines,
    const std::vector<StopLineSegment> &stop_lines,
    const std::vector<StopLineSegment> &end_lines);

struct VirtualTrafficLightArcInput {
  VirtualTrafficLightLineArcs arcs;
  double ego_arc_m = 0.0;
  double front_offset_m = 0.0;
  double dead_line_margin_m = 1.0;
  double near_line_distance_m = 1.0;
  double now_s = 0.0;
  bool has_stop_line = true;
  bool vehicle_stopped = false;
  std::optional<VirtualTrafficLightMessage> message;
};

// 차량 전면의 경로 위치와 선분 교차 호길이를 원본 상태 분기 입력으로 바꿉니다.
VirtualTrafficLightInput makeVirtualTrafficLightInputFromArcs(
    const VirtualTrafficLightArcInput &input);

}  // planning 네임스페이스
