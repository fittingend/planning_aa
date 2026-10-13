// Copyright 2022 Tier IV, Inc.
// SPDX-License-Identifier: Apache-2.0
// 원본 virtual_traffic_light/scene.cpp의 통과 권한 분기를 이관했습니다.

#include "planning/virtual_traffic_light_core.hpp"

#include <cmath>
#include <algorithm>
#include <stdexcept>
#include <vector>

namespace planning {

VirtualTrafficLightCore::VirtualTrafficLightCore(
    VirtualTrafficLightParameters parameters) : parameters_(parameters) {
  if (!std::isfinite(parameters_.maximum_message_delay_s) ||
      parameters_.maximum_message_delay_s < 0.0) {
    throw std::invalid_argument("가상 신호등 지연 설정이 잘못되었습니다");
  }
}

VirtualTrafficLightResult VirtualTrafficLightCore::apply(
    const VirtualTrafficLightInput &input) {
  if (!std::isfinite(input.now_s) ||
      (input.message && !std::isfinite(input.message->stamp_s))) {
    throw std::invalid_argument("가상 신호등 시각이 잘못되었습니다");
  }
  if (!input.has_end_line) return {state_, VirtualTrafficLightStop::None};
  if (input.before_start_line) {
    state_ = VirtualTrafficLightState::None;
    return {state_, VirtualTrafficLightStop::None};
  }
  if (input.after_end_line || state_ == VirtualTrafficLightState::Finalized) {
    state_ = VirtualTrafficLightState::Finalized;
    return {state_, VirtualTrafficLightStop::None};
  }

  state_ = VirtualTrafficLightState::Requesting;
  if (!input.has_stop_line) return {state_, VirtualTrafficLightStop::None};
  if (!input.message || !input.message->approved) {
    return {state_, VirtualTrafficLightStop::StopLine};
  }
  const bool timeout = input.now_s - input.message->stamp_s >
                       parameters_.maximum_message_delay_s;
  if (input.before_stop_line) {
    return {state_, timeout ? VirtualTrafficLightStop::StopLine
                            : VirtualTrafficLightStop::None};
  }

  state_ = VirtualTrafficLightState::Passing;
  if (parameters_.check_timeout_after_stop_line && timeout) {
    return {state_, VirtualTrafficLightStop::StopLine};
  }
  if (!input.message->finalized) {
    if (input.near_end_line && input.vehicle_stopped) {
      state_ = VirtualTrafficLightState::Finalizing;
    }
    return {state_, VirtualTrafficLightStop::EndLine};
  }
  return {state_, VirtualTrafficLightStop::None};
}

std::optional<VirtualTrafficLightCommand> makeVirtualTrafficLightCommand(
    const VirtualTrafficLightInput &input,
    const VirtualTrafficLightResult &decision,
    const VirtualTrafficLightCommandMetadata &metadata) {
  if (!std::isfinite(input.now_s))
    throw std::invalid_argument("가상 신호등 명령 시각이 잘못되었습니다");
  if (!input.has_end_line) return std::nullopt;
  return VirtualTrafficLightCommand{
      input.now_s, static_cast<std::uint8_t>(decision.state), metadata};
}

TrajectoryStopResult applyVirtualTrafficLightStopVelocity(
    const VirtualTrafficLightPathInput &input) {
  if (!std::isfinite(input.ego_arc_m) ||
      !std::isfinite(input.front_offset_m) ||
      !std::isfinite(input.hold_stop_margin_m) ||
      input.front_offset_m < 0.0 || input.hold_stop_margin_m < 0.0 ||
      (input.stop_line_arc_m && !std::isfinite(*input.stop_line_arc_m)) ||
      (input.end_line_arc_m && !std::isfinite(*input.end_line_arc_m))) {
    throw std::invalid_argument("가상 신호등 정지 경로 입력이 잘못되었습니다");
  }
  if (input.decision.stop == VirtualTrafficLightStop::None) {
    return {input.trajectory, std::nullopt};
  }
  if (input.decision.stop == VirtualTrafficLightStop::EndLine &&
      !input.end_line_arc_m && input.before_stop_line) {
    return {input.trajectory, std::nullopt};
  }
  const auto collision_arc = input.decision.stop ==
      VirtualTrafficLightStop::StopLine ? input.stop_line_arc_m :
      input.end_line_arc_m;
  double stop_arc = collision_arc.value_or(0.0) - input.front_offset_m;
  if (input.decision.stop == VirtualTrafficLightStop::StopLine &&
      collision_arc && input.vehicle_stopped &&
      stop_arc - input.ego_arc_m < input.hold_stop_margin_m) {
    stop_arc = input.ego_arc_m;
  }
  return insertZeroVelocityAtArc(input.trajectory, stop_arc);
}

VirtualTrafficLightLineArcs findVirtualTrafficLightLineArcs(
    const Trajectory &trajectory,
    const std::vector<StopLineSegment> &start_lines,
    const std::vector<StopLineSegment> &stop_lines,
    const std::vector<StopLineSegment> &end_lines) {
  for (const auto &point : trajectory) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
      throw std::invalid_argument("가상 신호등 경로 좌표에는 유한한 값이 필요합니다");
    }
  }
  for (const auto *lines : {&start_lines, &stop_lines, &end_lines}) {
    for (const auto &line : *lines) {
      if (!std::isfinite(line.x1) || !std::isfinite(line.y1) ||
          !std::isfinite(line.x2) || !std::isfinite(line.y2)) {
        throw std::invalid_argument("가상 신호등 선분 좌표에는 유한한 값이 필요합니다");
      }
    }
  }
  struct Crossing {
    double arc_m;
  };
  const auto crossings = [&](const std::vector<StopLineSegment> &lines) {
    std::vector<Crossing> found;
    double traveled = 0.0;
    for (std::size_t index = 0; index + 1 < trajectory.size(); ++index) {
      const auto &first = trajectory[index];
      const auto &second = trajectory[index + 1];
      const double dx = second.x - first.x;
      const double dy = second.y - first.y;
      const double length = std::hypot(dx, dy);
      if (length <= 1.0e-12) continue;
      for (const auto &line : lines) {
        const double lx = line.x2 - line.x1;
        const double ly = line.y2 - line.y1;
        const double denominator = dx * ly - dy * lx;
        if (std::abs(denominator) <= 1.0e-12) continue;
        const double ox = line.x1 - first.x;
        const double oy = line.y1 - first.y;
        const double path_ratio = (ox * ly - oy * lx) / denominator;
        const double line_ratio = (ox * dy - oy * dx) / denominator;
        if (path_ratio < -1.0e-9 || path_ratio > 1.0 + 1.0e-9 ||
            line_ratio < -1.0e-9 || line_ratio > 1.0 + 1.0e-9) continue;
        found.push_back({traveled + std::clamp(path_ratio, 0.0, 1.0) * length});
      }
      traveled += length;
    }
    return found;
  };
  VirtualTrafficLightLineArcs result;
  const auto ends = crossings(end_lines);
  if (ends.empty()) return result;
  result.end_arc_m = std::min_element(
      ends.begin(), ends.end(), [](const auto &a, const auto &b) {
        return a.arc_m < b.arc_m;
      })->arc_m;
  const auto last_before_end = [&](const std::vector<StopLineSegment> &lines) {
    std::optional<double> last;
    for (const auto crossing : crossings(lines)) {
      if (crossing.arc_m < *result.end_arc_m &&
          (!last || crossing.arc_m > *last)) last = crossing.arc_m;
    }
    return last;
  };
  result.start_arc_m = last_before_end(start_lines);
  result.stop_arc_m = last_before_end(stop_lines);
  return result;
}

VirtualTrafficLightInput makeVirtualTrafficLightInputFromArcs(
    const VirtualTrafficLightArcInput &input) {
  if (!std::isfinite(input.ego_arc_m) ||
      !std::isfinite(input.now_s) ||
      !std::isfinite(input.front_offset_m) ||
      !std::isfinite(input.dead_line_margin_m) ||
      !std::isfinite(input.near_line_distance_m) ||
      (input.arcs.start_arc_m && !std::isfinite(*input.arcs.start_arc_m)) ||
      (input.arcs.stop_arc_m && !std::isfinite(*input.arcs.stop_arc_m)) ||
      (input.arcs.end_arc_m && !std::isfinite(*input.arcs.end_arc_m)) ||
      input.front_offset_m < 0.0 || input.dead_line_margin_m < 0.0 ||
      input.near_line_distance_m < 0.0) {
    throw std::invalid_argument("가상 신호등 선분 위치 입력이 잘못되었습니다");
  }
  VirtualTrafficLightInput result;
  result.now_s = input.now_s;
  result.has_end_line = input.arcs.end_arc_m.has_value();
  result.has_stop_line = input.has_stop_line;
  result.vehicle_stopped = input.vehicle_stopped;
  result.message = input.message;
  const double front_arc = input.ego_arc_m + input.front_offset_m;
  result.before_start_line = input.arcs.start_arc_m &&
      *input.arcs.start_arc_m - front_arc > 0.0;
  result.before_stop_line = input.arcs.stop_arc_m &&
      *input.arcs.stop_arc_m - front_arc > -input.dead_line_margin_m;
  if (input.arcs.end_arc_m) {
    const double signed_end_distance = *input.arcs.end_arc_m - front_arc;
    result.after_end_line = !result.before_stop_line &&
        signed_end_distance < -input.dead_line_margin_m;
    result.near_end_line =
        std::abs(signed_end_distance) < input.near_line_distance_m;
  }
  return result;
}

}  // planning 네임스페이스
