// Copyright 2022 Tier IV, Inc.
// SPDX-License-Identifier: Apache-2.0
// 경로 호길이에 정지점을 삽입하고 이후 속도를 0으로 제한합니다.

#pragma once

#include "planning/trajectory.hpp"

#include <optional>

namespace planning {

struct TrajectoryStopResult {
  Trajectory trajectory;
  std::optional<double> stop_arc_m;
};

TrajectoryStopResult insertZeroVelocityAtArc(
    const Trajectory &trajectory, double stop_arc_m);

}  // planning 네임스페이스
