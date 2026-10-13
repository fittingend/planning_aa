// Copyright 2023 TIER IV, Inc.
// SPDX-License-Identifier: Apache-2.0
// 원본 traffic_light_utils.cpp의 차로별 신호 판정 규칙입니다.

#include "planning/traffic_light_signal.hpp"

#include <algorithm>

namespace planning {

bool trafficSignalRequiresStop(
    LaneTurnDirection lane_turn,
    const std::vector<TrafficLampElement> &elements) {
  const bool green_circle = std::any_of(
      elements.begin(), elements.end(), [](const auto &element) {
        return element.shape == TrafficLampShape::Circle &&
               element.color == TrafficLampColor::Green;
      });
  if (green_circle) return false;
  if (lane_turn == LaneTurnDirection::Else) return true;

  TrafficLampShape allowed_arrow = TrafficLampShape::Unknown;
  if (lane_turn == LaneTurnDirection::Right) {
    allowed_arrow = TrafficLampShape::RightArrow;
  } else if (lane_turn == LaneTurnDirection::Left) {
    allowed_arrow = TrafficLampShape::LeftArrow;
  } else if (lane_turn == LaneTurnDirection::Straight) {
    allowed_arrow = TrafficLampShape::UpArrow;
  }
  const bool arrow_present = std::any_of(
      elements.begin(), elements.end(), [allowed_arrow](const auto &element) {
        return element.shape == allowed_arrow;
      });
  return !arrow_present;
}

}  // planning 네임스페이스
