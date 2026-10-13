// SPDX-License-Identifier: Apache-2.0
// 하나의 예측 객체 묶음을 행동 속도 플러그인의 공통 입력으로 변환합니다.

#pragma once

#include "planning/behavior_velocity_map_planner.hpp"

#include <string>

namespace planning {

struct PlanningPerceivedObject {
  std::string id;
  std::uint8_t classification = 7;
  int shape_type = 0;
  double x = 0.0;
  double y = 0.0;
  double yaw_rad = 0.0;
  double length_m = 0.0;
  double width_m = 0.0;
  double velocity_x_mps = 0.0;
  double velocity_y_mps = 0.0;
  std::vector<CruisePredictedPose> footprint;
  std::vector<CruisePredictedPath> predicted_paths;
};

// 횡단보도·교차로·사각지대·돌발 진입이 동일한 객체 표본을 사용합니다.
void applyBehaviorVelocityPerceptionBatch(
    BehaviorVelocityMapInput &input,
    const std::vector<PlanningPerceivedObject> &objects);

}  // planning 네임스페이스
