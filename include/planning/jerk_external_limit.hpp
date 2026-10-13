// Copyright 2021 Tier IV, Inc.
// SPDX-License-Identifier: Apache-2.0
// 스무더 노드의 외부 속도 제한 변경 계산입니다.

#pragma once

#include "planning/jerk_node_policy.hpp"

namespace planning {

struct JerkExternalLimitParameters {
  double min_deceleration = -1.0;
  double max_jerk = 1.0;
  double min_jerk = -1.0;
  double insertion_margin = 0.3;
};

struct JerkExternalLimitPlan {
  double velocity_before_limit = 4.17;
  double target_velocity = 4.17;
  double insertion_distance = 0.0;
  bool distance_valid = true;
};

// 이전 출력 상태에서 새 외부 제한 속도까지의 감속 지점을 계산합니다.
JerkExternalLimitPlan planJerkExternalLimit(
    const JerkMotion &previous_motion, double target_velocity,
    const JerkExternalLimitParameters &parameters);

// 이전 콜백 위치에서 이동한 거리만큼 제한점까지 남은 거리를 줄입니다.
void advanceJerkExternalLimit(JerkExternalLimitPlan &plan, double travel_distance);

// 차량 위치에서 감속 거리만큼 앞에 제한점을 넣고 이후 속도를 제한합니다.
Trajectory applyJerkExternalLimit(
    const Trajectory &input, const TrajectoryPoint &ego_pose,
    const JerkExternalLimitPlan &plan);

}  // planning 네임스페이스
