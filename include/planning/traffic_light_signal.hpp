// Copyright 2023 TIER IV, Inc.
// SPDX-License-Identifier: Apache-2.0
// 신호등 인식 요소와 차로 회전 방향을 독립 자료형으로 분리합니다.

#pragma once

#include <vector>

namespace planning {

enum class LaneTurnDirection { Else, Left, Right, Straight };
enum class TrafficLampColor { Unknown, Red, Yellow, Green };
enum class TrafficLampShape { Unknown, Circle, LeftArrow, RightArrow, UpArrow };

struct TrafficLampElement {
  TrafficLampColor color = TrafficLampColor::Unknown;
  TrafficLampShape shape = TrafficLampShape::Unknown;
};

struct TrafficLightSignalObservation {
  double stamp_s = 0.0;
  LaneTurnDirection lane_turn = LaneTurnDirection::Else;
  std::vector<TrafficLampElement> elements;
};

bool trafficSignalRequiresStop(
    LaneTurnDirection lane_turn,
    const std::vector<TrafficLampElement> &elements);

}  // planning 네임스페이스
