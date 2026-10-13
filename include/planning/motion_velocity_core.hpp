// SPDX-License-Identifier: Apache-2.0
// 동작 속도 계획의 정지점·감속 구간을 궤적에 적용합니다.

#pragma once

#include "planning/trajectory.hpp"

#include <optional>
#include <vector>

namespace planning {

struct MotionVelocityPoint2D {
  double x = 0.0;
  double y = 0.0;
};

struct MotionVelocitySlowdown {
  MotionVelocityPoint2D from;
  MotionVelocityPoint2D to;
  double velocity = 0.0;
};

struct MotionVelocityPlan {
  std::vector<MotionVelocityPoint2D> stop_points;
  std::vector<MotionVelocitySlowdown> slowdowns;
};

// 연속 점 사이가 0.5m 이하이면 뒤 점을 생략합니다.
Trajectory resampleMotionVelocityPlanningInput(const Trajectory &input);

// 원본 노드와 같이 플러그인 순서대로 정지점·감속 구간을 삽입합니다.
Trajectory applyMotionVelocityPlans(
    const Trajectory &input, const std::vector<MotionVelocityPlan> &plans);

}  // planning 네임스페이스
