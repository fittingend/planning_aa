// Copyright 2020 Tier IV, Inc.
// SPDX-License-Identifier: Apache-2.0
// 행동 속도 계획 노드의 경로 정리와 재보간 계산입니다.

#pragma once

#include "planning/trajectory.hpp"

namespace planning {

Trajectory postprocessBehaviorPath(
    const Trajectory &path, double forward_length_m = 1000.0,
    double interval_m = 1.0);

}  // planning 네임스페이스
